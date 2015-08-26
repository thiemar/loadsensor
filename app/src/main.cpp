#include <stm32.h>
#include <irq.h>
#include "uavcan.h"
#include "can.h"
#include "configuration.h"
#include "fastmath.h"
#include "shared.h"


#include "uavcan/protocol/param/ExecuteOpcode.hpp"
#include "uavcan/protocol/param/GetSet.hpp"
#include "uavcan/protocol/file/BeginFirmwareUpdate.hpp"
#include "uavcan/protocol/GetNodeInfo.hpp"
#include "uavcan/protocol/NodeStatus.hpp"
#include "uavcan/protocol/RestartNode.hpp"
#include "uavcan/equipment/hardpoint/Status.hpp"


enum uavcan_dtid_filter_id_t {
    UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE = 0u,
    UAVCAN_PROTOCOL_PARAM_GETSET,
    UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE,
    UAVCAN_PROTOCOL_GETNODEINFO,
    UAVCAN_PROTOCOL_RESTARTNODE,
    UAVCAN_EQUIPMENT_HARDPOINT_STATUS
};


static volatile uint32_t g_uptime;
static struct bootloader_app_shared_t g_bootloader_app_shared;


/* Written to the firmware image in post-processing */
extern volatile struct bootloader_app_descriptor flash_app_descriptor;


static bool serial_read(int32_t *result);


static void node_init(uint8_t node_id);
static void node_run(uint8_t node_id, Configuration& configuration);


extern "C" void main(void) {
    up_cxxinitialize();
    board_initialize();
    irqenable();

    Configuration configuration;

    /*
    Read bootloader auto-baud and node ID values, then set up the node. We've
    just come from the bootloader, so CAN must already be configured
    correctly.
    */
    if (bootloader_read(&g_bootloader_app_shared)) {
        can_init(g_bootloader_app_shared.bus_speed);
        node_init(g_bootloader_app_shared.node_id);
        node_run(g_bootloader_app_shared.node_id, configuration);
    } else {
        /* Uh-oh */
        up_systemreset();
    }
}


extern "C" void sched_process_timer(void) {
    g_uptime++;
}


static void node_init(uint8_t node_id) {
    can_set_dtid_filter(
        1u, UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE, true,
        uavcan::protocol::param::ExecuteOpcode::DefaultDataTypeID,
        node_id);
    can_set_dtid_filter(
        1u, UAVCAN_PROTOCOL_PARAM_GETSET, true,
        uavcan::protocol::param::GetSet::DefaultDataTypeID,
        node_id);
    can_set_dtid_filter(
        1u, UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE, true,
        uavcan::protocol::file::BeginFirmwareUpdate::DefaultDataTypeID,
        node_id);
    can_set_dtid_filter(
        1u, UAVCAN_PROTOCOL_GETNODEINFO, true,
        uavcan::protocol::GetNodeInfo::DefaultDataTypeID,
        node_id);
    can_set_dtid_filter(
        1u, UAVCAN_PROTOCOL_RESTARTNODE, true,
        uavcan::protocol::RestartNode::DefaultDataTypeID,
        node_id);
}


static bool serial_read(int32_t *result) {
    uint8_t val, clocks;
    uint32_t temp;

    /*
    From the datasheet
    (https://cdn.sparkfun.com/datasheets/Sensors/ForceFlex/hx711_english.pdf):

    "Pin PD_SCK and DOUT are used for data retrieval, input selection, gain
    selection and power down controls.

    When output data is not ready for retrieval, digital output pin DOUT is
    high. Serial clock input PD_SCK should be low. When DOUT goes to low, it
    indicates data is ready for retrieval. By applying 25~27 positive clock
    pulses at the PD_SCK pin, data is shifted out from the DOUT output pin.
    Each PD_SCK pulse shifts out one bit, starting with the MSB bit first,
    until all 24 bits are shifted out. The 25th pulse at PD_SCK input will
    pull DOUT pin back to high (Fig.2).

    Input and gain selection is controlled by the number of the input PD_SCK
    pulses (Table 3). PD_SCK clock pulses should not be less than 25 or more
    than 27 within one conversion period, to avoid causing serial
    communication error."

    The minimum pulse width is 0.1 us, and the max is 50 us. We need to wait
    at least 0.1 us after DOUT goes low before we can transition PD_SCK high,
    but that's only 7 cycles so we'll always be waiting longer than that.
    */

    temp = 0u;

    /* If DOUT is high, data is NOT ready */
    val = stm32_gpioread(GPIO_DOUT);
    if (val) {
        return false;
    }

    for (clocks = 0; clocks < 25u; clocks++) {
        /*
        DOUT is high, so transition PD_SCK high and wait 1-2 us for the data
        to settle
        */
        stm32_gpiowrite(GPIO_PD_SCK, 1u);
        for (volatile uint8_t x = 0; x < 36u; x++);

        /* Sample the data */
        val = stm32_gpioread(GPIO_DOUT);
        temp = (temp << 1u) | (val ? 1u : 0u);

        /* Push PD_SCK low again, and wait a bit longer */
        stm32_gpiowrite(GPIO_PD_SCK, 0u);
        for (volatile uint8_t x = 0; x < 36u; x++);
    }

    /*
    We read 25 bits in the loop above, and the final bit is always one, so
    shift the output back down.
    */
    temp >>= 1u;

    /* Result is 2s-complement 24-bit, so sign-extend and convert to int32 */
    *result = (int32_t)(temp ^ 0x800000u) - 0x800000;

    return true;
}


static void __attribute__((noreturn)) node_run(
    uint8_t node_id,
    Configuration& configuration
) {
    size_t length, i;
    uint32_t message_id, current_time, status_time, hardpoint_time,
             status_interval, hardpoint_interval;
    int32_t sensor_data_lsb;
    uint8_t filter_id, hardpoint_transfer_id, status_transfer_id, message[8],
            hardpoint_id, service_filter_id;
    bool param_valid, wants_bootloader_restart;
    struct param_t param;
    float weight_n, sensor_offset_n, sensor_scale_n_per_lsb, value;

    UAVCANTransferManager broadcast_manager(node_id);
    UAVCANTransferManager service_manager(node_id);

    uavcan::protocol::param::ExecuteOpcode::Request xo_req;
    uavcan::protocol::param::GetSet::Request gs_req;
    uavcan::protocol::RestartNode::Request rn_req;

    hardpoint_transfer_id = status_transfer_id = 0u;
    hardpoint_time = status_time = 0u;

    service_filter_id = 0xFFu;

    wants_bootloader_restart = false;

    status_interval = 900u;
    hardpoint_interval = (uint32_t)(configuration.get_param_value_by_index(
        PARAM_UAVCAN_STATUS_INTERVAL) * 1e-3f);
    hardpoint_id = (uint8_t)configuration.get_param_value_by_index(
        PARAM_UAVCAN_HARDPOINT_ID);

    weight_n = 0.0f;
    sensor_offset_n =
        configuration.get_param_value_by_index(PARAM_SENSOR_OFFSET);
    sensor_scale_n_per_lsb =
        configuration.get_param_value_by_index(PARAM_SENSOR_SCALE) * 1e-3f;

    while (true) {
        current_time = g_uptime;

        /* Read the latest sensor data if available */
        if (serial_read(&sensor_data_lsb)) {
            weight_n = (float)sensor_data_lsb * sensor_scale_n_per_lsb -
                       sensor_offset_n;
        }

        /*
        Check for UAVCAN service requests (FIFO 1) -- only process if the
        first byte of the data is the local node ID
        */
        while (can_rx(1u, &filter_id, &message_id, &length, message)) {
            uavcan::TransferCRC crc;
            switch (filter_id) {
                case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE:
                    crc = uavcan::protocol::param::ExecuteOpcode::getDataTypeSignature().toTransferCRC();
                    break;
                case UAVCAN_PROTOCOL_PARAM_GETSET:
                    crc = uavcan::protocol::param::GetSet::getDataTypeSignature().toTransferCRC();
                    break;
                case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE:
                    crc = uavcan::protocol::file::BeginFirmwareUpdate::getDataTypeSignature().toTransferCRC();
                    break;
                case UAVCAN_PROTOCOL_GETNODEINFO:
                    crc = uavcan::protocol::GetNodeInfo::getDataTypeSignature().toTransferCRC();
                    break;
                case UAVCAN_PROTOCOL_RESTARTNODE:
                    crc = uavcan::protocol::RestartNode::getDataTypeSignature().toTransferCRC();
                    break;
                default:
                    break;
            }
            service_manager.receive_frame(current_time, message_id, crc,
                                          length, message);

            if (service_manager.is_rx_done()) {
                service_filter_id = filter_id;
                break;
            }
        }

        /*
        Don't process service requests until the last service response is
        completely sent, to avoid overwriting the TX buffer.
        */
        if (service_manager.is_rx_done() && service_manager.is_tx_done()) {
            if (service_filter_id == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE &&
                    service_manager.decode(xo_req)) {
                /*
                Return OK if the opcode is understood and the controller is
                stopped, otherwise reject.
                */
                uavcan::protocol::param::ExecuteOpcode::Response xo_resp;
                xo_resp.ok = false;
                if (xo_req.opcode == xo_req.OPCODE_SAVE) {
                    configuration.write_params();
                    xo_resp.ok = true;
                } else if (xo_req.opcode == xo_req.OPCODE_ERASE) {
                    /*
                    Set all parameters to default values, then erase the flash
                    */
                    for (i = 0u; i < NUM_PARAMS; i++) {
                        configuration.get_param_by_index(param, (uint8_t)i);
                        configuration.set_param_value_by_index(
                            (uint8_t)i, param.default_value);
                    }
                    configuration.write_params();
                    xo_resp.ok = true;
                }
                service_manager.encode_response<uavcan::protocol::param::ExecuteOpcode>(xo_resp);
            } else if (service_filter_id == UAVCAN_PROTOCOL_PARAM_GETSET &&
                    service_manager.decode(gs_req)) {
                uavcan::protocol::param::GetSet::Response resp;

                if (!gs_req.name.empty()) {
                    param_valid = configuration.get_param_by_name(
                        param, gs_req.name.c_str());
                } else {
                    param_valid = configuration.get_param_by_index(
                        param, (uint8_t)gs_req.index);
                }

                if (param_valid) {
                    if (param.public_type == PARAM_TYPE_FLOAT && !gs_req.name.empty() &&
                            gs_req.value.is(uavcan::protocol::param::Value::Tag::real_value)) {
                        value = gs_req.value.to<uavcan::protocol::param::Value::Tag::real_value>();
                        configuration.set_param_value_by_index(param.index,
                                                               value);
                    } else if (param.public_type == PARAM_TYPE_INT && !gs_req.name.empty() &&
                            gs_req.value.is(uavcan::protocol::param::Value::Tag::integer_value)) {
                        value = (float)((int32_t)gs_req.value.to<uavcan::protocol::param::Value::Tag::integer_value>());
                        configuration.set_param_value_by_index(param.index,
                                                               value);
                    }

                    value = configuration.get_param_value_by_index(
                        param.index);

                    resp.name = (const char*)param.name;
                    if (param.public_type == PARAM_TYPE_FLOAT) {
                        resp.value.to<uavcan::protocol::param::Value::Tag::real_value>() = value;
                        resp.default_value.to<uavcan::protocol::param::Value::Tag::real_value>() = param.default_value;
                        resp.min_value.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = param.min_value;
                        resp.max_value.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = param.max_value;
                    } else if (param.public_type == PARAM_TYPE_INT) {
                        resp.value.to<uavcan::protocol::param::Value::Tag::integer_value>() = (int32_t)value;
                        resp.default_value.to<uavcan::protocol::param::Value::Tag::integer_value>() = (int32_t)param.default_value;
                        resp.min_value.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = (int32_t)param.min_value;
                        resp.max_value.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = (int32_t)param.max_value;
                    }
                }

                service_manager.encode_response<uavcan::protocol::param::GetSet>(resp);
            } else if (service_filter_id == UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE) {
                uavcan::protocol::file::BeginFirmwareUpdate::Response resp;

                /*
                Don't actually need to decode since we don't care about the
                request data
                */
                resp.error = resp.ERROR_OK;
                wants_bootloader_restart = true;
                service_manager.encode_response<uavcan::protocol::file::BeginFirmwareUpdate>(resp);
            } else if (service_filter_id == UAVCAN_PROTOCOL_GETNODEINFO) {
                uavcan::protocol::GetNodeInfo::Response resp;

                /* Empty request so don't need to decode */
                resp.status.uptime_sec = current_time / 1000u;
                resp.status.health = resp.status.HEALTH_OK;
                resp.status.mode = resp.status.MODE_OPERATIONAL;
                resp.status.sub_mode = 0u;
                resp.status.vendor_specific_status_code = 0u;
                resp.software_version.major =
                    flash_app_descriptor.major_version;
                resp.software_version.minor =
                    flash_app_descriptor.minor_version;
                resp.software_version.optional_field_flags =
                    resp.software_version.OPTIONAL_FIELD_FLAG_VCS_COMMIT |
                    resp.software_version.OPTIONAL_FIELD_FLAG_IMAGE_CRC;
                resp.software_version.vcs_commit =
                    flash_app_descriptor.vcs_commit;
                resp.software_version.image_crc =
                    flash_app_descriptor.image_crc;
                resp.hardware_version.major = HW_VERSION_MAJOR;
                resp.hardware_version.minor = HW_VERSION_MINOR;
                /* Set the unique ID */
                memset(resp.hardware_version.unique_id.begin(), 0u,
                       resp.hardware_version.unique_id.size());
                memcpy(resp.hardware_version.unique_id.begin(),
                       (uint8_t*)0x1ffff7ac, 12u);
                /* Set the hardware name */
                resp.name = HW_UAVCAN_NAME;

                service_manager.encode_response<uavcan::protocol::GetNodeInfo>(resp);
            } else if (service_filter_id == UAVCAN_PROTOCOL_RESTARTNODE &&
                    service_manager.decode(rn_req)) {
                uavcan::protocol::RestartNode::Response resp;

                /*
                Restart if the magic number is correct, otherwise reject.
                */
                if (rn_req.magic_number == rn_req.MAGIC_NUMBER) {
                    resp.ok = true;
                    wants_bootloader_restart = true;
                } else {
                    resp.ok = false;
                }
                service_manager.encode_response<uavcan::protocol::RestartNode>(resp);
            }

            service_manager.receive_acknowledge();
        }

        /* Transmit service responses if available */
        if (can_is_ready(1u) &&
                service_manager.transmit_frame(message_id, length, message)) {
            can_tx(1u, message_id, length, message);
        }

        if (broadcast_manager.is_tx_done() && service_manager.is_tx_done() &&
                !service_manager.is_rx_in_progress(current_time)) {
            if (hardpoint_interval &&
                    current_time - hardpoint_time >= hardpoint_interval) {
                uavcan::equipment::hardpoint::Status msg;

                msg.payload_weight = weight_n;
                msg.payload_weight_variance = 1.0f;

                msg.hardpoint_id = hardpoint_id;

                broadcast_manager.encode_message(hardpoint_transfer_id++, msg);
                hardpoint_time = current_time;
            } else if (current_time - status_time >= status_interval) {
                uavcan::protocol::NodeStatus msg;

                msg.uptime_sec = current_time / 1000u;
                msg.health = msg.HEALTH_OK;
                msg.mode = msg.MODE_OPERATIONAL;
                msg.sub_mode = 0u;
                msg.vendor_specific_status_code = 0u;
                broadcast_manager.encode_message(status_transfer_id++, msg);
                status_time = current_time;
            }
        }

        /* Transmit broadcast CAN frames if available */
        if (can_is_ready(0u) &&
                broadcast_manager.transmit_frame(message_id, length, message)) {
            can_tx(0u, message_id, length, message);
        }

        /*
        Only restart into the bootloader if the acknowledgement message has
        been sent.
        */
        if (broadcast_manager.is_tx_done() && service_manager.is_tx_done() &&
                wants_bootloader_restart) {
            up_systemreset();
        }
    }
}
