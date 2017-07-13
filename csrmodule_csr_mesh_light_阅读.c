/******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2014-2015
 *  CSR Bluetooth Low Energy CSRmesh 1.2 Release
 *  Application Version 1.2
 *
 *  FILE
 *      csr_mesh_light.c
 *
 *  DESCRIPTION
 *      This file implements the CSR Mesh light application.
 *
 *****************************************************************************/

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/
#include <main.h>
#include <ls_app_if.h>
#include <gatt.h>
#include <timer.h>
#include <uart.h>
#include <pio.h>
#include <nvm.h>
#include <security.h>
#include <gatt_prim.h>
#include <mem.h>
#include <panic.h>
#include <config_store.h>
#include <random.h>
#include <buf_utils.h>

/*============================================================================*
 *  Local Header Files
 *============================================================================*/
#include "user_config.h"
#include "nvm_access.h"
#include "csr_mesh_light.h"
#include "app_debug.h"
#include "app_gatt.h"
#include "csr_mesh_light_hw.h"
#include "gap_service.h"
#include "app_gatt_db.h"
#include "mesh_control_service.h"
#include "csr_mesh_light_gatt.h"
#include "appearance.h"

#if defined (ENABLE_GATT_OTA_SERVICE) || defined (ENABLE_FIRMWARE_MODEL)
#include "csr_ota.h"
#include "csr_ota_service.h"
#include "gatt_service.h"
#endif /* ENABLE_GATT_OTA_SERVICE || ENABLE_FIRMWARE_MODEL */

#ifdef USE_ASSOCIATION_REMOVAL_KEY
#include "iot_hw.h"
#endif /* USE_ASSOCIATION_REMOVAL_KEY */
#include "battery_hw.h"
#include "app_data_stream.h"

/*============================================================================*
 *  CSR Mesh Header Files
 *============================================================================*/
#include <csr_mesh.h>
#include <attention_model.h>
#include <light_model.h>
#include <power_model.h>
#include <bearer_model.h>
#include <ping_model.h>
#include <debug.h>

#ifdef ENABLE_FIRMWARE_MODEL
#include <firmware_model.h>
#endif
#ifdef ENABLE_BATTERY_MODEL
#include <battery_model.h>
#endif

/*============================================================================*
 *  Private Definitions
 *============================================================================*/
/* Association Removal Button Press Duration */
#define LONG_KEYPRESS_TIME             (2 * SECOND)

/* CSRmesh Device UUID size */
#define DEVICE_UUID_SIZE_WORDS          8

/* CSRmesh Authorization Code Size in Words */
#define DEVICE_AUTHCODE_SIZE_IN_WORDS  (4)

/* CS Key for user flags */
#define CSKEY_INDEX_USER_FLAGS         (CSR_MESH_CS_USERKEY_INDEX_ADV_TIME + 1)

/* Used for generating Random UUID */
#define RANDOM_UUID_ENABLE_MASK        (0x0001)

/* Used for permanently Enabling/Disabling Relay */
#define RELAY_ENABLE_MASK              (0x0002)

/* Used for permanently Enabling/Disabling Bridge */
#define BRIDGE_ENABLE_MASK             (0x0004)

/* OTA Reset Defer Duration */
#define OTA_RESET_DEFER_DURATION       (500 * MILLISECOND)

/* NVM Data Write defer Duration */
#define NVM_WRITE_DEFER_DURATION       (5 * SECOND)

#define MAX_APP_TIMERS                 (10 + MAX_CSR_MESH_TIMERS)

/* Advertisement Timer for sending device identification */
#define DEVICE_ID_ADVERT_TIME          (5 * SECOND)

/* Slave device is not allowed to transmit another Connection Parameter
 * Update request till time TGAP(conn_param_timeout). Refer to section 9.3.9.2,
 * Vol 3, Part C of the Core 4.0 BT spec. The application should retry the
 * 'connection parameter update' procedure after time TGAP(conn_param_timeout)
 * which is 30 seconds.
 */
 / *从设备不允许传输另一个连接参数
  *更新请求直到时间TGAP（conn_param_timeout）。 参见9.3.9.2节，
  *第3卷，Core 4.0 BT规格的C部分。 应用程序应该在30秒的TGAP（conn_param_timeout）
  *后'连接参数更新'程序，。
 */
#define GAP_CONN_PARAM_TIMEOUT         (30 * SECOND)


/* TGAP(conn_pause_peripheral) defined in Core Specification Addendum 3 Revision
 * 2. A Peripheral device should not perform a Connection Parameter Update procedure
 *  within TGAP(conn_pause_peripheral) after establishing a connection.
 */
/*
    TGAP 由核心规范附录3版本修订2版中定义的。一个外设不应该在
    它建立一个连接后的TGAP时间内，执行连接参数更新程序
*/
#define TGAP_CPP_PERIOD                (1 * SECOND)

/* TGAP(conn_pause_central) defined in Core Specification Addendum 3 Revision 2.
 * After the Peripheral device has no further pending actions to perform and the
 * Central device has not initiated any other actions within TGAP(conn_pause_ce-
 * -ntral), then the Peripheral device may perform a Connection Parameter Update
 * procedure.
 */
 /*
    核心规范附录3修订版2中定义的TGAP.
    当外设没有进一步的动作执行，且中心设备在TGAP时间内尚未初始化其它操作，则外设
    可以执行 连接参数更新 程序。
 */
#define TGAP_CPC_PERIOD                (1 * SECOND)

/* NVM magic version used by the 1.1 application */
#define NVM_SANITY_MAGIC_1_1           (0xAB18)

/* APP NVM version used by the 1.1 light application */
#define APP_NVM_VERSION_1_1            (2)

/* Magic value to check the sanity of NVM region used by the application. This 
 * value should be unique for each application as the NVM layout changes for
 * every application.
 */
 /*
    应用程序用来检测NVM区域的合理值。
    此值在每个应用程序中，都是独一无二的。正如NVM布局因每个不同的应用而变动
 */
#define NVM_SANITY_MAGIC               (0xAB81)

/*Number of IRKs that application can store */
#define MAX_NUMBER_IRK_STORED          (1)

/* NVM offset for the application NVM version */
#define NVM_OFFSET_SANITY_WORD         (0)

/* NVM offset for NVM sanity word */
#define NVM_OFFSET_APP_NVM_VERSION     (NVM_OFFSET_SANITY_WORD + 1)

/* NVM offset for CSRmesh device UUID */
#define NVM_OFFSET_DEVICE_UUID         (NVM_OFFSET_APP_NVM_VERSION + 1)

/* NVM Offset for Authorization Code */
#define NVM_OFFSET_DEVICE_AUTHCODE     (NVM_OFFSET_DEVICE_UUID + \
                                        DEVICE_UUID_SIZE_WORDS)

/* NVM Offset for CSRmesh Library */
#define NVM_OFFSET_CSRMESH_LIB         (NVM_OFFSET_DEVICE_AUTHCODE + \
                                        DEVICE_AUTHCODE_SIZE_IN_WORDS)

/* Number of words of NVM used by application. Memory used by supported
 * services is not taken into consideration here.
 */
 /*
    应用程序使用的NVM的字节数。
    支撑服务的内存没有考虑在内。--- q_cl 此处 service 指的是什么？
 */
#define NVM_OFFSET_ASSOCIATION_STATE   (NVM_OFFSET_CSRMESH_LIB + \
                                        CSR_MESH_NVM_SIZE)

#define NVM_OFFSET_LIGHT_MODEL_GROUPS  (NVM_OFFSET_ASSOCIATION_STATE + 1)

#define NVM_OFFSET_POWER_MODEL_GROUPS  (NVM_OFFSET_LIGHT_MODEL_GROUPS + \
                                        sizeof(light_model_groups))

#define NVM_OFFSET_ATTENTION_MODEL_GROUPS \
                                       (NVM_OFFSET_POWER_MODEL_GROUPS + \
                                        sizeof(power_model_groups))
#ifdef ENABLE_DATA_MODEL                                        
#define NVM_OFFSET_DATA_MODEL_GROUPS \
                                       (NVM_OFFSET_ATTENTION_MODEL_GROUPS + \
                                        sizeof(attention_model_groups))

/* NVM Offset for RGB data */
#define NVM_RGB_DATA_OFFSET            (NVM_OFFSET_DATA_MODEL_GROUPS + \
                                        sizeof(data_model_groups))
#else
                                       
/* NVM Offset for RGB data */
#define NVM_RGB_DATA_OFFSET            (NVM_OFFSET_ATTENTION_MODEL_GROUPS + \
                                        sizeof(attention_model_groups))
#endif                                       

/* Size of RGB Data in Words */
#define NVM_RGB_DATA_SIZE              (2)

/* NVM offset for Bearer Model Data */
#define NVM_BEARER_DATA_OFFSET         (NVM_RGB_DATA_OFFSET + \
                                        NVM_RGB_DATA_SIZE)

/* NVM Offset for Application data */
#define NVM_MAX_APP_MEMORY_WORDS       (NVM_BEARER_DATA_OFFSET + \
                                        sizeof(BEARER_MODEL_STATE_DATA_T))

/*============================================================================*
 *  Public Data
 *============================================================================*/
/* CSRmesh light application specific data */
 /* CSRmesh 灯应用所用的数据 */
CSRMESH_LIGHT_DATA_T g_lightapp_data;

/* Application VID,PID and Version. */
CSR_MESH_VID_PID_VERSION_T vid_pid_info =
{
    .vendor_id  = APP_VENDOR_ID,
    .product_id = APP_PRODUCT_ID,
    .version    = APP_VERSION,
};

/* Device Apprearance. */
CSR_MESH_APPEARANCE_T device_appearance = {APPEARANCE_ORG_BLUETOOTH_SIG,
                                           APPEARANCE_CSRMESH_LIGHT_VALUE};

/* Device Short name */
/* 设备的名称 */
uint8 short_name[9] = "Light";

/*============================================================================*
 *  Private Data
 *============================================================================*/
/* CSRmesh Device UUID: 128-Bit Device UUID is stored in 8 Words(16-Bits each)
 * in Little Endian Format in RAM and NVM. To set a particular UUID, change the
 * light_uuid array as shown in example.
 * Example:
 * ............................MSB.................................LSB.
 * Device UUID in Hexadecimal: 0123-4567-89AB-CDEF-FEDC-BA98-7654-3210.
 * Array should be set as: {0x3210, 0x7654, 0xBA98, 0xFEDC,
 *                          0xCDEF, 0x89AB, 0x4567, 0x0123};
 * NOTE: The UUID in array below will be used only if, Bit-0 of
 * CSKEY_INDEX_USER_FLAGS is Cleared.
 */
 /*
    CSRmesh设备UUID：一个128比特位的设备UUID存储在8个字当中（每个字16位，128 = 16 * 8）
    以小端的形式存储在RAM 和 NVM 中。为了得到正常的UUID，要像下面例子那样更改 light_uuid 数组
    注：下面数组中的UUID，只会在 CSKEY_INDEX_USER_FLAGS 的 BIT-0 位被清除。
 */
static uint16 light_uuid[DEVICE_UUID_SIZE_WORDS] =
                                    {0x3210, 0x7654, 0xBA98, 0xFEDC,
                                     0xCDEF, 0x89AB, 0x4567, 0x0123};

#ifdef USE_AUTHORIZATION_CODE
/* CSRmesh Device Authorization Code: 64-Bit Device Authorization Code is
 * stored in 4 Words(16-Bits each) in Little Endian Format in RAM and NVM.
 * To set a particular Authorization Code, change the
 * light_auth_code array as shown in example.
 * Example:
 * ..........................................MSB.............LSB.
 * Device Authorization Code in Hexadecimal: 0123-4567-89AB-CDEF.
 * Array should be set as: {0xCDEF, 0x89AB, 0x4567, 0x0123};
 */
 /*
    CSRmesh设备的授权码：64比特位，4个字，小端表示法存储
 */
static uint16 light_auth_code[DEVICE_AUTHCODE_SIZE_IN_WORDS] =
                                            {0xCDEF, 0x89AB, 0x4567, 0x0123};
#endif /* USE_AUTHORIZATION_CODE */


/* Declare space for application timers. */
static uint16 app_timers[SIZEOF_APP_TIMER * MAX_APP_TIMERS];

/* Declare space for Model Groups */
static uint16 light_model_groups[MAX_MODEL_GROUPS];
static uint16 attention_model_groups[MAX_MODEL_GROUPS];
static uint16 power_model_groups[MAX_MODEL_GROUPS];
#ifdef ENABLE_DATA_MODEL
static uint16 data_model_groups[MAX_MODEL_GROUPS];
#endif /* ENABLE_DATA_MODEL */

#ifdef USE_ASSOCIATION_REMOVAL_KEY
/* Association Button Press Timer */
static timer_id long_keypress_tid;
#endif /* USE_ASSOCIATION_REMOVAL_KEY */

/* Attention timer id */
static timer_id attn_tid = TIMER_INVALID;

#ifdef ENABLE_FIRMWARE_MODEL
/* Firmware Reset Delay Timer Id */
static timer_id ota_rst_tid = TIMER_INVALID;
#endif /* ENABLE_FIRMWARE_MODEL */

/* To send the MASP associate to NW msg and Dev appearance msg alternatively */
static bool send_dev_appearance = FALSE;

/*============================================================================*
 *  Private Function Prototypes
 *============================================================================*/
/* UART Receive callback */
#ifdef DEBUG_ENABLE
static uint16 UartDataRxCallback ( void* p_data, uint16 data_count,
                                   uint16* p_num_additional_words );
#endif /* DEBUG_ENABLE */

#ifdef USE_ASSOCIATION_REMOVAL_KEY
static void handlePIOEvent(pio_changed_data *data);
#endif /* USE_ASSOCIATION_REMOVAL_KEY */

/* Advert time out handler */
static void smLightDeviceIdAdvertTimeoutHandler(timer_id tid);
/*2017-5-9  此处只是申明这个函数，在后面实现*/
/* This function reads the persistent store. */
static void readPersistentStore(void);

/*============================================================================*
 *  Private Function Definitions
 *============================================================================*/
#ifdef USE_STATIC_RANDOM_ADDRESS
/*-----------------------------------------------------------------------------*
 *  NAME
 *      generateStaticRandomAddress
 *
 *  DESCRIPTION
 *      This function generates a static random address.
 *
 *  RETURNS/MODIFIES
 *      Nothing
 *
 *----------------------------------------------------------------------------*/
static void generateStaticRandomAddress(BD_ADDR_T *addr)
{
    uint16 temp[3];
    uint16 idx = 0;

    if (!addr) return;

    for (idx = 0; idx < 3;)
    {
        temp[idx] = Random16();
        if ((temp[idx] != 0) && (temp[idx] != 0xFFFF))
        {
            idx++;
        }
    }

    addr->lap = ((uint32)(temp[1]) << 16) | (temp[0]);
    addr->lap &= 0x00FFFFFFUL;
    addr->uap = (temp[1] >> 8) & 0xFF;
    addr->nap = temp[2];

    addr->nap &= ~BD_ADDR_NAP_RANDOM_TYPE_MASK;
    addr->nap |=  BD_ADDR_NAP_RANDOM_TYPE_STATIC;
}
#endif /* USE_STATIC_RANDOM_ADDRESS */

#ifdef ENABLE_FIRMWARE_MODEL
/*-----------------------------------------------------------------------------*
 *  NAME
 *      issueOTAReset
 *
 *  DESCRIPTION
 *      This function issues an OTA Reset.
 *
 *  RETURNS/MODIFIES
 *      Nothing
 *
 *----------------------------------------------------------------------------*/
static void issueOTAReset(timer_id tid)
{
    if (ota_rst_tid == tid)
    {
        ota_rst_tid = TIMER_INVALID;

        /* Issue OTA Reset. */
        OtaReset();
    }
}
#endif /* ENABLE_FIRMWARE_MODEL */

/*-----------------------------------------------------------------------------*
 *  NAME
 *      smLightDeviceIdAdvertTimeoutHandler
 *
 *  DESCRIPTION
 *      This function handles the Device ID advertise timer event.
 *
 *  RETURNS/MODIFIES
 *      Nothing
 *
 *----------------------------------------------------------------------------*/
 /*-----------------------------------------------------------------------------*
  * 名称
  * smLightDeviceIdAdvertTimeoutHandler
 *
  *说明
  *此函数处理 设备ID广播 定时器事件。
 *
  *返回/修改
  * 无
 *
 *----------------------------------------------------------------------------*/
static void smLightDeviceIdAdvertTimeoutHandler(timer_id tid)
{
    if(tid == g_lightapp_data.mesh_device_id_advert_tid)
    {
        if(g_lightapp_data.assoc_state == app_state_not_associated)
        {   
            // 灯泡未绑定
            /* Generate a random delay between 0 to 511 ms */
            uint32 random_delay = ((uint32)(Random16() & 0x1FF)) * (MILLISECOND);

            // 灯泡不处于快速广播阶段
            if (g_lightapp_data.state != app_state_fast_advertising)
            {
                // 未发送设备表征信息
                if(send_dev_appearance == FALSE)
                {
                /* Send the device ID advertisements */
                CsrMeshAssociateToANetwork();
                    send_dev_appearance = TRUE;
                }
                else
                {
                /* Send the device appearance */
                // q_cl：发送设备表征信息，可用，发送其它信息？ 
                CsrMeshAdvertiseDeviceAppearance(&device_appearance, 
                                                 short_name, 
                                                 sizeof(short_name));
                    send_dev_appearance = FALSE;
            }
            }

            g_lightapp_data.mesh_device_id_advert_tid = TimerCreate(
                                         (DEVICE_ID_ADVERT_TIME + random_delay),
                                         TRUE,
                                         smLightDeviceIdAdvertTimeoutHandler);
        }
        else
        {
            /* Device is now associated so no need to start the timer again */
            g_lightapp_data.mesh_device_id_advert_tid = TIMER_INVALID;
        }
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      smLightDataNVMWriteTimerHandler
 *
 *  DESCRIPTION
 *      This function handles NVM Write Timer time-out.
 *      该函数处理NVM写入事件定时器 （超时事件） --- 定时向 NVM 写入数据
 *  RETURNS/MODIFIES
 *      Nothing
 *
 *----------------------------------------------------------------------------*/
static void smLightDataNVMWriteTimerHandler(timer_id tid)
{
    uint32 rd_data = 0;
    uint32 wr_data = 0;

    if (tid == g_lightapp_data.nvm_tid)
    {
        g_lightapp_data.nvm_tid = TIMER_INVALID;

        /* Read RGB and Power Data from NVM */
        Nvm_Read((uint16 *)&rd_data, sizeof(uint32),
                 NVM_RGB_DATA_OFFSET);

        /* Pack Data for writing to NVM */
        wr_data = ((uint32) g_lightapp_data.power.power_state << 24) |
                  ((uint32) g_lightapp_data.light_state.blue  << 16) |
                  ((uint32) g_lightapp_data.light_state.green <<  8) |
                  g_lightapp_data.light_state.red;

        /* If data on NVM is not equal to current state, write current state
         * to NVM.
         */
        if (rd_data != wr_data)
        {
            Nvm_Write((uint16 *)&wr_data, sizeof(uint32),NVM_RGB_DATA_OFFSET);
        }
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      attnTimerHandler
 *
 *  DESCRIPTION
 *      This function handles Attention time-out.
 *
 *  RETURNS/MODIFIES
 *      Nothing
 *
 *----------------------------------------------------------------------------*/
static void attnTimerHandler(timer_id tid)
{
    if (attn_tid == tid)
    {
        attn_tid = TIMER_INVALID;

        /* Restore Light State */
        LightHardwareSetColor(g_lightapp_data.light_state.red,
                              g_lightapp_data.light_state.green,
                              g_lightapp_data.light_state.blue);

        LightHardwarePowerControl(g_lightapp_data.power.power_state);
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      smAppInitiateAssociation
 *
 *  DESCRIPTION
 *      This function starts timer to send CSRmesh Association Messages
 *      and also gives visual indication that light is not associated.

 *      启动一个定时器：用来发送 设备绑定信息，并发出该 灯未被绑定的暗示信号 -- 灯光蓝色并闪烁

 *  RETURNS/MODIFIES
 *      Nothing
 *
 *----------------------------------------------------------------------------*/
static void smAppInitiateAssociation(void)
{
    /* Generate a random delay between 0 to 511 ms */
    uint32 random_delay = ((uint32)(Random16() & 0x1FF)) * (MILLISECOND);

    /* Blink light to indicate that it is not associated 未绑定时的闪烁的颜色*/
    LightHardwareSetBlink(0, 0, 127, 32, 32);

    /* Start a timer to send Device ID messages periodically to get
     * associated to a network
     */
     /*
        启动一个定时器，用来周期发送设备ID信息，以便跟网络绑定上
        q_cl : device ID 怎么由设备自己确定？ a_cl : 从前面 line:426 可知，应该不
        包含 deviceID 信息，只是自己的表征
     */
    g_lightapp_data.mesh_device_id_advert_tid =
                    TimerCreate((random_delay + DEVICE_ID_ADVERT_TIME),
                                TRUE,
                                smLightDeviceIdAdvertTimeoutHandler);
}


/*-----------------------------------------------------------------------------*
 *  NAME
 *      togglePowerState
 *
 *  DESCRIPTION
 *      This function toggles the power state.
 *
 *  RETURNS/MODIFIES
 *      Nothing
 *
 *----------------------------------------------------------------------------*/
static void togglePowerState(void)
{
    POWER_STATE_T curr_state = g_lightapp_data.power.power_state;

    switch (curr_state)
    {
        case POWER_STATE_ON:
            g_lightapp_data.power.power_state = POWER_STATE_OFF;
        break;

        case POWER_STATE_OFF:
            g_lightapp_data.power.power_state = POWER_STATE_ON;
        break;

        case POWER_STATE_ON_FROM_STANDBY:
            g_lightapp_data.power.power_state = POWER_STATE_STANDBY;
        break;

        case POWER_STATE_STANDBY:
            g_lightapp_data.power.power_state = POWER_STATE_ON_FROM_STANDBY;
        break;

        default:
        break;
    }
}


#ifdef USE_ASSOCIATION_REMOVAL_KEY
/*-----------------------------------------------------------------------------*
 *  NAME
 *      longKeyPressTimeoutHandler
 *
 *  DESCRIPTION
 *      This function handles the long key press timer event.
 *
 *  RETURNS/MODIFIES
 *      Nothing
 *
 *----------------------------------------------------------------------------*/
static void longKeyPressTimeoutHandler(timer_id tid)
{
    if (long_keypress_tid == tid)
    {
        long_keypress_tid = TIMER_INVALID;
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      handlePIOEvent
 *
 *  DESCRIPTION
 *      This function handles the PIO Events.
 *
 *  RETURNS/MODIFIES
 *      Nothing
 *
 *----------------------------------------------------------------------------*/
void handlePIOEvent(pio_changed_data *data)
{
    /* If Switch-2 related event, then process further. Otherwise ignore */
    /*如果Switch-2相关事件，则进一步处理。 否则忽略*/
    if (data->pio_cause & SW2_MASK)
    {
        /* Button Pressed */
        if (!(data->pio_state & SW2_MASK))
        {
            TimerDelete(long_keypress_tid);

            long_keypress_tid = TimerCreate(LONG_KEYPRESS_TIME, TRUE,
                                            longKeyPressTimeoutHandler);
        }
        else /* Button Released */
        {
            /* Button released after long press */
            if (TIMER_INVALID == long_keypress_tid)
            {
                if (app_state_not_associated != g_lightapp_data.assoc_state)
                {
                    /* Reset Association Information */
                    CsrMeshReset();

                    /* Set state to un-associated */
                    g_lightapp_data.assoc_state = app_state_not_associated;

                    /* Write association state to NVM */
                    Nvm_Write((uint16 *)&g_lightapp_data.assoc_state,
                             sizeof(g_lightapp_data.assoc_state),
                             NVM_OFFSET_ASSOCIATION_STATE);
                }
            }
            else /* Button released after a short press */
            {
                if (app_state_not_associated == g_lightapp_data.assoc_state)
                {
                    /* Delete Long Key Press Timer */
                    TimerDelete(long_keypress_tid);

                    /* Start Association to CSRmesh */
                    smAppInitiateAssociation();
                }
            }
        }
    }
}
#endif /* USE_ASSOCIATION_REMOVAL_KEY */

/*----------------------------------------------------------------------------*
 *  NAME
 *      requestConnParamUpdate
 *
 *  DESCRIPTION
 *      This function is used to send L2CAP_CONNECTION_PARAMETER_UPDATE_REQUEST
 *      to the remote device when an earlier sent request had failed.
 *      
        该函数用来发送 L2CAP_..._REQUEST 到远程设备上，当之前的请求发送失败
                                            q_cl : --- 指的是哪个？
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void requestConnParamUpdate(timer_id tid)
{
    /* Application specific preferred parameters */
    ble_con_params app_pref_conn_param;

    if(g_lightapp_data.gatt_data.con_param_update_tid == tid)
    {

        g_lightapp_data.gatt_data.con_param_update_tid = TIMER_INVALID;
        g_lightapp_data.gatt_data.cpu_timer_value = 0;

        /*Handling signal as per current state */
        switch(g_lightapp_data.state)
        {

            case app_state_connected:
            {
                /* Increment the count for Connection Parameter Update
                 * requests
                 */
                 // 每次连接参数更新都自增一次
                ++ g_lightapp_data.gatt_data.num_conn_update_req;

                /* If it is first or second request, preferred connection
                 * parameters should be request
                 */
                 // 前两次请求，应该请求的首选参数
                if(g_lightapp_data.gatt_data.num_conn_update_req == 1 ||
                   g_lightapp_data.gatt_data.num_conn_update_req == 2)
                {
                    app_pref_conn_param.con_max_interval =
                                                PREFERRED_MAX_CON_INTERVAL;
                    app_pref_conn_param.con_min_interval =
                                                PREFERRED_MIN_CON_INTERVAL;
                    app_pref_conn_param.con_slave_latency =
                                                PREFERRED_SLAVE_LATENCY;
                    app_pref_conn_param.con_super_timeout =
                                                PREFERRED_SUPERVISION_TIMEOUT;
                }
                /* If it is 3rd or 4th request, APPLE compliant parameters
                 * should be requested.
                 */
                 // 第三、四次请求时，应该选择与APPLE兼容性参数
                else if(g_lightapp_data.gatt_data.num_conn_update_req == 3 ||
                        g_lightapp_data.gatt_data.num_conn_update_req == 4)
                {
                    app_pref_conn_param.con_max_interval =
                                                APPLE_MAX_CON_INTERVAL;
                    app_pref_conn_param.con_min_interval =
                                                APPLE_MIN_CON_INTERVAL;
                    app_pref_conn_param.con_slave_latency =
                                                APPLE_SLAVE_LATENCY;
                    app_pref_conn_param.con_super_timeout =
                                                APPLE_SUPERVISION_TIMEOUT;
                }

                /* Send Connection Parameter Update request using application
                 * specific preferred connection parameters
                 */
                 // 使用应用具体的首选参数
                if(LsConnectionParamUpdateReq(
                                 &g_lightapp_data.gatt_data.con_bd_addr,
                                 &app_pref_conn_param) != ls_err_none)
                {
                    ReportPanic(app_panic_con_param_update);
                }
            }
            break;

            default:
                /* Ignore in other states */
            break;
        }

    } /* Else ignore the timer */

}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      handleGapCppTimerExpiry
 *
 *  DESCRIPTION
 *      This function handles the expiry of TGAP(conn_pause_peripheral) timer.
 *      It starts the TGAP(conn_pause_central) timer, during which, if no activ-
 *      -ity is detected from the central device, a connection parameter update
 *      request is sent.
 *
        该函数处理TGAP定时器的过期。该函数启动一个TGAP定时器，如果没有检测到主设备
        的活动，一个 连接参数更新 请求 就会被发送出去
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

static void handleGapCppTimerExpiry(timer_id tid)
{
    if(g_lightapp_data.gatt_data.con_param_update_tid == tid)
    {
        g_lightapp_data.gatt_data.con_param_update_tid =
                           TimerCreate(TGAP_CPC_PERIOD, TRUE,
                                       requestConnParamUpdate);
        g_lightapp_data.gatt_data.cpu_timer_value = TGAP_CPC_PERIOD;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      smLightDataInit
 *
 *  DESCRIPTION
 *      This function is called to initialise CSRmesh light application
 *      data structure.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void smLightDataInit(void)
{

    /* Reset/Delete all the timers */
    TimerDelete(g_lightapp_data.gatt_data.app_tid);
    g_lightapp_data.gatt_data.app_tid = TIMER_INVALID;

    TimerDelete(g_lightapp_data.gatt_data.con_param_update_tid);
    g_lightapp_data.gatt_data.con_param_update_tid = TIMER_INVALID;
    g_lightapp_data.gatt_data.cpu_timer_value = 0;

    g_lightapp_data.gatt_data.st_ucid = GATT_INVALID_UCID;
    g_lightapp_data.gatt_data.advert_timer_value = 0;

    /* Reset the connection parameter variables. */
    g_lightapp_data.gatt_data.conn_interval = 0;
    g_lightapp_data.gatt_data.conn_latency = 0;
    g_lightapp_data.gatt_data.conn_timeout = 0;

    /* Initialise GAP Data structure */
    GapDataInit();

    /* Initialize the Mesh Control Service Data Structure */
    // 初始化 Mesh 控制服务数据结构
    MeshControlServiceDataInit();

#ifdef ENABLE_GATT_OTA_SERVICE
    /* Initialise GATT Data structure */
    GattDataInit();

    /* Initialise the CSR OTA Service Data */
    OtaDataInit();
#endif /* ENABLE_GATT_OTA_SERVICE */
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      readPersistentStore
 *
 *  DESCRIPTION
 *      This function is used to initialize and read NVM data
 *
     // 初始化并读取 NVM 数据 
 *  RETURNS/MODIFIES
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
static void readPersistentStore(void)
{
    /* NVM offset for supported services */
    uint16 nvm_offset = 0;
    uint16 nvm_sanity = 0xffff;
    uint16 app_nvm_version = 0;
    uint32 temp = 0;

    nvm_offset = NVM_MAX_APP_MEMORY_WORDS;

    /* Read the sanity word */
    Nvm_Read(&nvm_sanity, sizeof(nvm_sanity),
             NVM_OFFSET_SANITY_WORD);

    /* Read the Application NVM version */
    Nvm_Read(&app_nvm_version, 1, NVM_OFFSET_APP_NVM_VERSION);

    if(nvm_sanity == NVM_SANITY_MAGIC &&
       app_nvm_version == APP_NVM_VERSION )
    {

       g_lightapp_data.gatt_data.paired = FALSE;

        /* Read RGB and Power Data from NVM */
        Nvm_Read((uint16 *)&temp, sizeof(uint32),
                 NVM_RGB_DATA_OFFSET);

        /* Read assigned Group IDs for Light model from NVM */
        Nvm_Read((uint16 *)light_model_groups, sizeof(light_model_groups),
                                                 NVM_OFFSET_LIGHT_MODEL_GROUPS);

        /* Read assigned Group IDs for Power model from NVM */
        Nvm_Read((uint16 *)power_model_groups, sizeof(power_model_groups),
                                                 NVM_OFFSET_POWER_MODEL_GROUPS);

        /* Read assigned Group IDs for Attention model from NVM */
        Nvm_Read((uint16 *)attention_model_groups, sizeof(attention_model_groups),
                                            NVM_OFFSET_ATTENTION_MODEL_GROUPS);
#ifdef ENABLE_DATA_MODEL
        /* Read assigned Group IDs for data stream model from NVM */
        Nvm_Read((uint16 *)data_model_groups, sizeof(data_model_groups),
                                            NVM_OFFSET_DATA_MODEL_GROUPS);
#endif /* ENABLE_DATA_MODEL */

        /* Unpack data in to the global variables */
        g_lightapp_data.light_state.red   = temp & 0xFF;
        temp >>= 8;
        g_lightapp_data.light_state.green = temp & 0xFF;
        temp >>= 8;
        g_lightapp_data.light_state.blue  = temp & 0xFF;
        temp >>= 8;
        g_lightapp_data.power.power_state = temp & 0xFF;
        g_lightapp_data.light_state.power = g_lightapp_data.power.power_state;

        /* Read Bearer Model Data from NVM */
        Nvm_Read((uint16 *)&g_lightapp_data.bearer_data,
                 sizeof(BEARER_MODEL_STATE_DATA_T), NVM_BEARER_DATA_OFFSET);

        /* If NVM in use, read device name and length from NVM */
        GapReadDataFromNVM(&nvm_offset);
    }
    else
    {
        /* Read Configuration flags from User CS Key */
        uint16 cskey_flags = CSReadUserKey(CSKEY_INDEX_USER_FLAGS);

        /* If the NVM_SANITY word the APP_NVM_VERSION matches the 1.1 version
         * retain the association data on the NVM and update the versions to
         * 1.2
         */
        if(nvm_sanity == NVM_SANITY_MAGIC_1_1 && 
            app_nvm_version == APP_NVM_VERSION_1_1)
        {
            nvm_sanity = NVM_SANITY_MAGIC;

            /* Write NVM Sanity word to the NVM */
            Nvm_Write(&nvm_sanity, sizeof(nvm_sanity), NVM_OFFSET_SANITY_WORD);
        }
        
        if(nvm_sanity != NVM_SANITY_MAGIC)
        {
            uint16 nvm_invalid_value = 0;
            uint8 i;

            /* NVM Sanity check failed means either the device is being brought
             * up for the first time or memory has got corrupted in which case
             * discard the data and start fresh.
             */
            nvm_sanity = NVM_SANITY_MAGIC;

            /* Write NVM Sanity word to the NVM */
            Nvm_Write(&nvm_sanity, sizeof(nvm_sanity),
                      NVM_OFFSET_SANITY_WORD);

            if (cskey_flags & RANDOM_UUID_ENABLE_MASK)
            {
                /* The flag is set so generate a random UUID and store it on NVM */
                for( i = 0 ; i < DEVICE_UUID_SIZE_WORDS ; i++)
                {
                    light_uuid[i] = Random16();
                }
                /* Write to NVM */
                Nvm_Write(light_uuid, DEVICE_UUID_SIZE_WORDS,
                          NVM_OFFSET_DEVICE_UUID);
            }
            else
            {
                /* A mass production tool may be used to program the device
                 * firmware as well as the device UUID and authorisation code
                 * in which case the NVM sanity check fails but the NVM will
                 * have a valid UUID and Authorisation code.
                 * So write the default UUID and AC only if all the words of the
                 *
                 */
                uint16 temp_word;
                for( i = 0; i < DEVICE_UUID_SIZE_WORDS ; i++)
                {
                    Nvm_Read(&temp_word, 1, NVM_OFFSET_DEVICE_UUID + i);
                    if( temp_word != NVM_DEFAULT_ERASED_WORD)
                    {
                        /* Atleast one word is not same default erased word
                         * read the UUID from NVM
                         */
                        Nvm_Read(light_uuid, DEVICE_UUID_SIZE_WORDS,
                                 NVM_OFFSET_DEVICE_UUID);
                        break;
                    }
                }

                if( i == DEVICE_UUID_SIZE_WORDS)
                {
                    /* Write the default UUID to NVM */
                    Nvm_Write(light_uuid, DEVICE_UUID_SIZE_WORDS,
                              NVM_OFFSET_DEVICE_UUID);
                }
            }
#ifdef USE_AUTHORIZATION_CODE
            uint16 temp_word;
            for( i = 0; i < DEVICE_AUTHCODE_SIZE_IN_WORDS ; i++)
            {
                Nvm_Read(&temp_word, 1, NVM_OFFSET_DEVICE_AUTHCODE + i);
                if( temp_word != NVM_DEFAULT_ERASED_WORD)
                {
                    Nvm_Read(light_auth_code, DEVICE_AUTHCODE_SIZE_IN_WORDS,
                                                    NVM_OFFSET_DEVICE_AUTHCODE);
                    break;
                }
            }
            if( i == DEVICE_AUTHCODE_SIZE_IN_WORDS)
            {
                /* Write default authorization Code to NVM */
                Nvm_Write(light_auth_code, DEVICE_AUTHCODE_SIZE_IN_WORDS,
                           NVM_OFFSET_DEVICE_AUTHCODE);
            }
#endif /* USE_AUTHORIZATION_CODE */

            /* Erase the Mesh Library information as sanity word has changed */
            for(i=0; i < CSR_MESH_NVM_SIZE; i++)
            {
                Nvm_Write(&nvm_invalid_value, sizeof(nvm_invalid_value),
                    NVM_OFFSET_CSRMESH_LIB + i);
            }

            /* The device will not be associated as it is coming up for the
             * first time
             */
            g_lightapp_data.assoc_state = app_state_not_associated;

            /* Write association state to NVM */
            Nvm_Write((uint16 *)&g_lightapp_data.assoc_state,
                     sizeof(g_lightapp_data.assoc_state),
                     NVM_OFFSET_ASSOCIATION_STATE);
        }

        /* Store new version of the NVM */
        app_nvm_version = APP_NVM_VERSION;
        Nvm_Write(&app_nvm_version, 1, NVM_OFFSET_APP_NVM_VERSION);

        /* All the persistent data below will be reset to default upon an
         * application update. If some of the data needs to be retained even
         * after an application update, it has to be moved within the sanity
         * word check
         */

        /* Update Bearer Model Data from CSKey flags for the first time. */
        g_lightapp_data.bearer_data.promiscuous       = FALSE;
        g_lightapp_data.bearer_data.bearerEnabled     = BLE_BEARER_MASK;
        g_lightapp_data.bearer_data.bearerRelayActive = 0x0000;

        if (cskey_flags & RELAY_ENABLE_MASK)
        {
            g_lightapp_data.bearer_data.bearerRelayActive |= BLE_BEARER_MASK;
        }

        if (cskey_flags & BRIDGE_ENABLE_MASK)
        {
            g_lightapp_data.bearer_data.bearerEnabled |=     BLE_GATT_SERVER_BEARER_MASK;
            g_lightapp_data.bearer_data.bearerRelayActive |= BLE_GATT_SERVER_BEARER_MASK;
        }

        /* Update Bearer Model Data to NVM */
        Nvm_Write((uint16 *)&g_lightapp_data.bearer_data,
                  sizeof(BEARER_MODEL_STATE_DATA_T), NVM_BEARER_DATA_OFFSET);

        /* The device will not be paired as it is coming up for the first
         * time
         */
        g_lightapp_data.gatt_data.paired = FALSE;

        /* Write RGB Data and Power to NVM.
         * Data is stored in the following format.
         * HIGH WORD: MSB: POWER LSB: BLUE.
         * LOW  WORD: MSB: GREEN LSB: RED.
         */
        g_lightapp_data.light_state.red   = 0xFF;
        g_lightapp_data.light_state.green = 0xFF;
        g_lightapp_data.light_state.blue  = 0xFF;
        g_lightapp_data.light_state.power = POWER_STATE_OFF;
        g_lightapp_data.power.power_state = POWER_STATE_OFF;

        temp = ((uint32) g_lightapp_data.power.power_state << 24) |
               ((uint32) g_lightapp_data.light_state.blue  << 16) |
               ((uint32) g_lightapp_data.light_state.green <<  8) |
               g_lightapp_data.light_state.red;

        Nvm_Write((uint16 *)&temp, sizeof(uint32),
                 NVM_RGB_DATA_OFFSET);

        /* Initialize model groups */
        MemSet(light_model_groups, 0x0000, sizeof(light_model_groups));
        Nvm_Write((uint16 *)light_model_groups, sizeof(light_model_groups),
                                                 NVM_OFFSET_LIGHT_MODEL_GROUPS);

        MemSet(power_model_groups, 0x0000, sizeof(power_model_groups));
        Nvm_Write((uint16 *)power_model_groups, sizeof(power_model_groups),
                                                 NVM_OFFSET_POWER_MODEL_GROUPS);

        MemSet(attention_model_groups, 0x0000, sizeof(attention_model_groups));
        Nvm_Write((uint16 *)attention_model_groups,
                  sizeof(attention_model_groups), NVM_OFFSET_ATTENTION_MODEL_GROUPS);

        /* Data stream model */
#ifdef ENABLE_DATA_MODEL
        MemSet(data_model_groups, 0x0000, sizeof(data_model_groups));
        Nvm_Write((uint16 *)data_model_groups, sizeof(data_model_groups),
                                        NVM_OFFSET_DATA_MODEL_GROUPS);
#endif /* ENABLE_DATA_MODEL */
        /* Write device name and length to NVM for the first time */
        GapInitWriteDataToNVM(&nvm_offset);
    }

    /* Read association state from NVM */
    Nvm_Read((uint16 *)&g_lightapp_data.assoc_state,
            sizeof(g_lightapp_data.assoc_state), NVM_OFFSET_ASSOCIATION_STATE);

    /* Read the UUID from NVM */
    Nvm_Read(light_uuid, DEVICE_UUID_SIZE_WORDS, NVM_OFFSET_DEVICE_UUID);

#ifdef USE_AUTHORIZATION_CODE
    /* Read Authorization Code from NVM */
    Nvm_Read(light_auth_code, DEVICE_AUTHCODE_SIZE_IN_WORDS,
             NVM_OFFSET_DEVICE_AUTHCODE);
#endif /* USE_AUTHORIZATION_CODE */

}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      UartDataRxCallback
 *
 *  DESCRIPTION
 *      This callback is issued when data is received over UART. Application
 *      may ignore the data, if not required. For more information refer to
 *      the API documentation for the type "uart_data_out_fn"

 *      当从 UART -- 通用异步传输器 中接受到数据会被调用。如果不需要，应用程序可能会忽略掉该数据。
        更多的信息，可以从 API 文档中获取。
 *  RETURNS
 *      The number of words processed, return data_count if all of the received
 *      data had been processed (or if application don't care about the data)
 *
 *----------------------------------------------------------------------------*/

#ifdef DEBUG_ENABLE
static uint16 UartDataRxCallback ( void* p_data, uint16 data_count,
        uint16* p_num_additional_words )
{

    /* Application needs 1 additional data to be received */
    *p_num_additional_words = 1;

    return data_count;
}
#endif /* DEBUG_ENABLE */


/*----------------------------------------------------------------------------*
 *  NAME
 *      appAdvertisingExit
 *
 *  DESCRIPTION
 *      This function is called while exiting app_state_fast_advertising and
 *      app_state_slow_advertising states.
        当退出快速与慢速广播状态
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void appAdvertisingExit(void)
{
        /* Cancel advertisement timer */
        TimerDelete(g_lightapp_data.gatt_data.app_tid);
        g_lightapp_data.gatt_data.app_tid = TIMER_INVALID;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalGattAddDBCfm
 *
 *  DESCRIPTION
 *      This function handles the signal GATT_ADD_DB_CFM
        该函数用于处理 GATT_ADD_DB_CFM 信号 
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void handleSignalGattAddDBCfm(GATT_ADD_DB_CFM_T *p_event_data)
{
    switch(g_lightapp_data.state)
    {
        case app_state_init:
        {
            if(p_event_data->result == sys_status_success)
            {
                /* Always do slow adverts on GATT Connection */
                AppSetState(app_state_fast_advertising);
            }
            else
            {
                /* Don't expect this to happen */
                ReportPanic(app_panic_db_registration);
            }
        }
        break;

        default:
            /* Control should never come here */
            ReportPanic(app_panic_invalid_state);
        break;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalGattCancelConnectCfm
 *
 *  DESCRIPTION
 *      This function handles the signal GATT_CANCEL_CONNECT_CFM
        该函数处理GATT_CANCEL_CONNECT_CFM信号
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void handleSignalGattCancelConnectCfm(void)
{
    /*Handling signal as per current state */
    switch(g_lightapp_data.state)
    {
        case app_state_fast_advertising:
        {
            /* Do nothing here */
        }
        break;

        case app_state_slow_advertising:
            /* There is no idle state, the device
             * will advertise for ever
             */
        break;

        case app_state_connected:
            /* The CSRmesh could have been sending data on
             * advertisements so do not panic
             */
        break;

        case app_state_idle:
        break;

        default:
            /* Control should never come here */
            ReportPanic(app_panic_invalid_state);
        break;
    }
}

/*---------------------------------------------------------------------------
 *
 *  NAME
 *      handleSignalLmEvConnectionComplete
 *
 *  DESCRIPTION
 *      This function handles the signal LM_EV_CONNECTION_COMPLETE.
 *
 *  RETURNS
 *      Nothing.
 *

*----------------------------------------------------------------------------*/
static void handleSignalLmEvConnectionComplete(
                                     LM_EV_CONNECTION_COMPLETE_T *p_event_data)
{
    /* Store the connection parameters. */
    g_lightapp_data.gatt_data.conn_interval = p_event_data->data.conn_interval;
    g_lightapp_data.gatt_data.conn_latency  = p_event_data->data.conn_latency;
    g_lightapp_data.gatt_data.conn_timeout  =
                                        p_event_data->data.supervision_timeout;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalGattConnectCfm
 *
 *  DESCRIPTION
 *      This function handles the signal GATT_CONNECT_CFM
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void handleSignalGattConnectCfm(GATT_CONNECT_CFM_T* p_event_data)
{
    /*Handling signal as per current state */
    switch(g_lightapp_data.state)
    {
        case app_state_fast_advertising:
        case app_state_slow_advertising:
        {
            if(p_event_data->result == sys_status_success)
            {
                /* Store received UCID */
                g_lightapp_data.gatt_data.st_ucid = p_event_data->cid;

                /* Store connected BD Address */
                g_lightapp_data.gatt_data.con_bd_addr = p_event_data->bd_addr;

                /* Enter connected state */
                AppSetState(app_state_connected);

                /* Inform CSRmesh that we are connected now */
                // 通知 CSRmesh ，我们正在尝试连接
                CsrMeshHandleDataInConnection(
                                g_lightapp_data.gatt_data.st_ucid,
                                g_lightapp_data.gatt_data.conn_interval);

                /* Start Listening in CSRmesh */
                CsrMeshStart();

                /* Since CSRmesh application does not mandate encryption
                 * requirement on its characteristics, the remote master may
                 * or may not encrypt the link. Start a timer  here to give
                 * remote master some time to encrypt the link and on expiry
                 * of that timer, send a connection parameter update request
                 * to remote side.
                 */
                 // 由于CSRmesh应用程序不要求对其属性进行加密，远程主设备可能加密也可能不加密链接。
                 // 这里启动一个定时器，用来给远程主设备一些时间去加密链接，当定时器时间到了，则会
                 // 发送给远程设备一个连接参数更新请求

                /* Don't request security as this causes connection issues
                 * with Android 4.4
                 *
                 * SMRequestSecurityLevel(&g_lightapp_data.gatt_data.con_bd_addr);
                 */

                /* If the current connection parameters being used don't
                 * comply with the application's preferred connection
                 * parameters and the timer is not running and, start timer
                 * to trigger Connection Parameter Update procedure
                 */
                 // 当当前的连接参数不被应用的首选参数所所用，且定时器没有启动
                 // 开始定时器将会触发连接参数更新程序
                if((g_lightapp_data.gatt_data.con_param_update_tid ==
                                                        TIMER_INVALID) &&
                   (g_lightapp_data.gatt_data.conn_interval <
                                             PREFERRED_MIN_CON_INTERVAL ||
                    g_lightapp_data.gatt_data.conn_interval >
                                             PREFERRED_MAX_CON_INTERVAL
#if PREFERRED_SLAVE_LATENCY
                    || g_lightapp_data.gatt_data.conn_latency <
                                             PREFERRED_SLAVE_LATENCY
#endif
                   )
                  )
                {
                    /* Set the num of conn update attempts to zero */
                    // 将更新尝试次数设置为 0 
                    g_lightapp_data.gatt_data.num_conn_update_req = 0;

                    /* The application first starts a timer of
                     * TGAP_CPP_PERIOD. During this time, the application
                     * waits for the peer device to do the database
                     * discovery procedure. After expiry of this timer, the
                     * application starts one more timer of period
                     * TGAP_CPC_PERIOD. If the application receives any
                     * GATT_ACCESS_IND during this time, it assumes that
                     * the peer device is still doing device database
                     * discovery procedure or some other configuration and
                     * it should not update the parameters, so it restarts
                     * the TGAP_CPC_PERIOD timer. If this timer expires, the
                     * application assumes that database discovery procedure
                     * is complete and it initiates the connection parameter
                     * update procedure.
                     */
                     // 应用首先启动 TGAP_CPP_PERIOD 定时器。
                     // 该定时期间，应用等待配对设备执行数据库发现程序。
                    // 该定时器结束的时候，应用开启一个或者更多的 TGAP_CPC_PERIOD 周期。
                    // 如果在此期间，应用接受到任何 GATT_ACCESS_IND 信号，则假定配对设备
                    // 仍在执行数据库发现程序，或者其它配置，应用不应该进行连接参数更新。
                    // 因此，应用重新启动 TGAP_CPC_PERIOD 定时器。
                    // 当这个定时器过期，应用认为数据库发现过程执行完成，并初始化连接参数更新。
                    g_lightapp_data.gatt_data.con_param_update_tid =
                                      TimerCreate(TGAP_CPP_PERIOD, TRUE,
                                                  handleGapCppTimerExpiry);
                    g_lightapp_data.gatt_data.cpu_timer_value =
                                                        TGAP_CPP_PERIOD;
                }
                  /* Else at the expiry of timer Connection parameter
                   * update procedure will get triggered
                   */
            }
            else
            {
                /* We don't use slow advertising. So, switch device
                 * to fast adverts.
                 */
                AppSetState(app_state_fast_advertising);
            }
        }
        break;

        default:
            /* Control should never come here */
            ReportPanic(app_panic_invalid_state);
        break;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalSmSimplePairingCompleteInd
 *
 *  DESCRIPTION
 *      This function handles the signal SM_SIMPLE_PAIRING_COMPLETE_IND
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void handleSignalSmSimplePairingCompleteInd(
                                 SM_SIMPLE_PAIRING_COMPLETE_IND_T *p_event_data)
{
    /*Handling signal as per current state */
    switch(g_lightapp_data.state)
    {
        case app_state_connected:
        {
            if(p_event_data->status == sys_status_success)
            {
                /* Store temporary pairing info. */
                g_lightapp_data.gatt_data.paired = TRUE;
                    }
            else
            {
                /* Pairing has failed.disconnect the link.*/
                AppSetState(app_state_disconnecting);
            }
        }
        break;

        default:
            /* Firmware may send this signal after disconnection. So don't
             * panic but ignore this signal.
             */
        break;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalLsConnParamUpdateCfm
 *
 *  DESCRIPTION
 *      This function handles the signal LS_CONNECTION_PARAM_UPDATE_CFM.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void handleSignalLsConnParamUpdateCfm(
                            LS_CONNECTION_PARAM_UPDATE_CFM_T *p_event_data)
{
    /*Handling signal as per current state */
    switch(g_lightapp_data.state)
    {
        case app_state_connected:
        {
            /* Received in response to the L2CAP_CONNECTION_PARAMETER_UPDATE
              * request sent from the slave after encryption is enabled. If
              * the request has failed, the device should again send the same
              * request only after Tgap(conn_param_timeout). Refer
              * Bluetooth 4.0 spec Vol 3 Part C, Section 9.3.9 and profile spec.
              */
              // 从设备允许加密后，发送请求。如果请求失败，设备在 Tgap 时间到后，再次发送。
            if ((p_event_data->status != ls_err_none) &&
                (g_lightapp_data.gatt_data.num_conn_update_req <
                                        MAX_NUM_CONN_PARAM_UPDATE_REQS))
            {
                /* Delete timer if running */
                TimerDelete(g_lightapp_data.gatt_data.con_param_update_tid);

                g_lightapp_data.gatt_data.con_param_update_tid =
                                 TimerCreate(GAP_CONN_PARAM_TIMEOUT,
                                             TRUE, requestConnParamUpdate);
                g_lightapp_data.gatt_data.cpu_timer_value =
                                             GAP_CONN_PARAM_TIMEOUT;
            }
        }
        break;

        default:
            /* Control should never come here */
            ReportPanic(app_panic_invalid_state);
        break;
    }
}


/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalLmConnectionUpdate
 *
 *  DESCRIPTION
 *      This function handles the signal LM_EV_CONNECTION_UPDATE.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void handleSignalLmConnectionUpdate(
                                   LM_EV_CONNECTION_UPDATE_T* p_event_data)
{
    switch(g_lightapp_data.state)
    {
        case app_state_connected:
        case app_state_disconnecting:
        {
            /* Store the new connection parameters. */
            g_lightapp_data.gatt_data.conn_interval =
                                            p_event_data->data.conn_interval;
            g_lightapp_data.gatt_data.conn_latency =
                                            p_event_data->data.conn_latency;
            g_lightapp_data.gatt_data.conn_timeout =
                                        p_event_data->data.supervision_timeout;

            CsrMeshHandleDataInConnection(g_lightapp_data.gatt_data.st_ucid,
                                       g_lightapp_data.gatt_data.conn_interval);

            DEBUG_STR("Parameter Update Complete: ");
            DEBUG_U16(g_lightapp_data.gatt_data.conn_interval);
            DEBUG_STR("\r\n");
        }
        break;

        default:
            /* Connection parameter update indication received in unexpected
             * application state.
             */
            ReportPanic(app_panic_invalid_state);
        break;
    }
}


/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalLsConnParamUpdateInd
 *
 *  DESCRIPTION
 *      This function handles the signal LS_CONNECTION_PARAM_UPDATE_IND.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void handleSignalLsConnParamUpdateInd(
                                 LS_CONNECTION_PARAM_UPDATE_IND_T *p_event_data)
{
    /*Handling signal as per current state */
    switch(g_lightapp_data.state)
    {
        case app_state_connected:
        {
            /* Delete timer if running */
            TimerDelete(g_lightapp_data.gatt_data.con_param_update_tid);
            g_lightapp_data.gatt_data.con_param_update_tid = TIMER_INVALID;
            g_lightapp_data.gatt_data.cpu_timer_value = 0;

            /* The application had already received the new connection
             * parameters while handling event LM_EV_CONNECTION_UPDATE.
             * Check if new parameters comply with application preferred
             * parameters. If not, application shall trigger Connection
             * parameter update procedure
             */

            if(g_lightapp_data.gatt_data.conn_interval <
                                                PREFERRED_MIN_CON_INTERVAL ||
               g_lightapp_data.gatt_data.conn_interval >
                                                PREFERRED_MAX_CON_INTERVAL
#if PREFERRED_SLAVE_LATENCY
               || g_lightapp_data.gatt_data.conn_latency <
                                                PREFERRED_SLAVE_LATENCY
#endif
              )
            {
                /* Set the num of conn update attempts to zero */
                g_lightapp_data.gatt_data.num_conn_update_req = 0;

                /* Start timer to trigger Connection Parameter Update
                 * procedure
                 */
                g_lightapp_data.gatt_data.con_param_update_tid =
                                TimerCreate(GAP_CONN_PARAM_TIMEOUT,
                                            TRUE, requestConnParamUpdate);
                g_lightapp_data.gatt_data.cpu_timer_value =
                                                        GAP_CONN_PARAM_TIMEOUT;
            }
        }
        break;

        default:
            /* Control should never come here */
            ReportPanic(app_panic_invalid_state);
        break;
    }

}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalGattAccessInd
 *
 *  DESCRIPTION
 *      This function handles GATT_ACCESS_IND message for attributes
 *      maintained by the application.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void handleSignalGattAccessInd(GATT_ACCESS_IND_T *p_event_data)
{

    /*Handling signal as per current state */
    switch(g_lightapp_data.state)
    {
        case app_state_connected:
        {
            /* GATT_ACCESS_IND indicates that the central device is still disco-
             * -vering services. So, restart the connection parameter update
             * timer
             */
             // GATT_ACCESS_IND 表明中心设备仍然在发现服务。所以重启连接参数更新定时器
             if(g_lightapp_data.gatt_data.cpu_timer_value == TGAP_CPC_PERIOD &&
                g_lightapp_data.gatt_data.con_param_update_tid != TIMER_INVALID)
             {
                TimerDelete(g_lightapp_data.gatt_data.con_param_update_tid);
                g_lightapp_data.gatt_data.con_param_update_tid =
                                    TimerCreate(TGAP_CPC_PERIOD,
                                                TRUE, requestConnParamUpdate);
             }

            /* Received GATT ACCESS IND with write access */
            if(p_event_data->flags & ATT_ACCESS_WRITE)
            {
                /* If only ATT_ACCESS_PERMISSION flag is enabled, then the
                 * firmware is asking the app for permission to go along with
                 * prepare write request from the peer. Allow it.
                 */
                 // 如果只有 ATT_ACCESS_PERMISSION 标志使能，则固件会请求程序允许准备
                 // 来自对等体的写请求。

                if(((p_event_data->flags) &
                   (ATT_ACCESS_PERMISSION | ATT_ACCESS_WRITE_COMPLETE))
                                                    == ATT_ACCESS_PERMISSION)
                {
                    GattAccessRsp(p_event_data->cid, p_event_data->handle,
                                  sys_status_success, 0, NULL);
                }
                else
                {
                    HandleAccessWrite(p_event_data);
                }
            }
            else if(p_event_data->flags & ATT_ACCESS_WRITE_COMPLETE)
            {
                GattAccessRsp(p_event_data->cid, p_event_data->handle,
                                          sys_status_success, 0, NULL);
            }
            /* Received GATT ACCESS IND with read access */
            else if(p_event_data->flags ==
                                    (ATT_ACCESS_READ | ATT_ACCESS_PERMISSION))
            {
                HandleAccessRead(p_event_data);
            }
            else
            {
                GattAccessRsp(p_event_data->cid, p_event_data->handle,
                              gatt_status_request_not_supported,
                              0, NULL);
            }
        }
        break;

        default:
            /* Control should never come here */
            ReportPanic(app_panic_invalid_state);
        break;
    }
}


/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalLmDisconnectComplete
 *
 *  DESCRIPTION
 *      This function handles LM Disconnect Complete event which is received
 *      at the completion of disconnect procedure triggered either by the
 *      device or remote host or because of link loss
        //该函数处理 LM 丢失完成事件，当接受到断开连接程序完成信号。无论该断开连接程序
        //是由设备自己还是远程主机亦或信仅只是连接丢失触发的
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void handleSignalLmDisconnectComplete(
                HCI_EV_DATA_DISCONNECT_COMPLETE_T *p_event_data)
{

    /* Reset the connection parameter variables. */
    g_lightapp_data.gatt_data.conn_interval = 0;
    g_lightapp_data.gatt_data.conn_latency = 0;
    g_lightapp_data.gatt_data.conn_timeout = 0;

    CsrMeshHandleDataInConnection(GATT_INVALID_UCID, 0);
#ifdef ENABLE_GATT_OTA_SERVICE
    if(OtaResetRequired())
    {
        OtaReset();
    }
#endif /* ENABLE_GATT_OTA_SERVICE */
    /* LM_EV_DISCONNECT_COMPLETE event can have following disconnect
     * reasons:
     *
     * HCI_ERROR_CONN_TIMEOUT - Link Loss case
     * HCI_ERROR_CONN_TERM_LOCAL_HOST - Disconnect triggered by device
     * HCI_ERROR_OETC_* - Other end (i.e., remote host) terminated connection
     */
     // 断开连接的三种可能
    /*Handling signal as per current state */
    switch(g_lightapp_data.state)
    {
        case app_state_connected:
        case app_state_disconnecting:
        {
            /* Connection is terminated either due to Link Loss or
             * the local host terminated connection. In either case
             * initialize the app data and go to fast advertising.
             */
             // 非本设备的原因断开连接，重启程序数据
            smLightDataInit();
            AppSetState(app_state_fast_advertising);
        }
        break;

        default:
            /* Control should never come here */
            ReportPanic(app_panic_invalid_state);
        break;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleCsrMeshGroupSetMsg
 *
 *  DESCRIPTION
 *      This function handles the CSRmesh Group Assignment message. Stores
 *      the group_id at the given index for the model
        // 该函数处理 CSRmesh 分组 信号。在模型指定的索引处储存 group_id 
        // q_cl: 一次只能发送一个模型，更改一个模型的分组。是吗，与具体的信号是怎么产生的
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static bool handleCsrMeshGroupSetMsg(uint8 *msg, uint16 len)
{
    CSR_MESH_MODEL_TYPE_T model = msg[0];
    uint8 index = msg[1];
    uint16 group_id = msg[3] + (msg[4] << 8);
    bool update_lastetag = TRUE;

    /* In case an incorrect index is received return without updating grps. */
    if(index >= MAX_MODEL_GROUPS)
    {
        return FALSE;
    }

    switch(model)
    {
        case CSR_MESH_LIGHT_MODEL:
        {
            /* Store Group ID */
            light_model_groups[index] = group_id;

            /* Save to NVM */
            Nvm_Write(&light_model_groups[index],
                     sizeof(uint16),
                     NVM_OFFSET_LIGHT_MODEL_GROUPS + index);
        }
        break;

        case CSR_MESH_POWER_MODEL:
        {
            power_model_groups[index] = group_id;

            /* Save to NVM */
            Nvm_Write(&power_model_groups[index],
                     sizeof(uint16),
                     NVM_OFFSET_POWER_MODEL_GROUPS + index);
        }
        break;

        case CSR_MESH_ATTENTION_MODEL:
        {
            attention_model_groups[index] = group_id;

            /* Save to NVM */
            Nvm_Write(&attention_model_groups[index],
                     sizeof(uint16),
                     NVM_OFFSET_ATTENTION_MODEL_GROUPS + index);
        }
        break;

#ifdef ENABLE_DATA_MODEL
        case CSR_MESH_DATA_MODEL:
        {
            data_model_groups[index] = group_id;

            /* Save to NVM */
            Nvm_Write(&data_model_groups[index],
                     sizeof(uint16),
                     NVM_OFFSET_DATA_MODEL_GROUPS + index);
        }
        break;
#endif /* ENABLE_DATA_MODEL */

        default:
            update_lastetag = FALSE;
        break;
    }

    return update_lastetag;
}

#ifdef ENABLE_BACKOFF
/*----------------------------------------------------------------------------*
 *  NAME
 *      backoffEventHandler
 *
 *  DESCRIPTION
 *      This function handles the CSRmesh back-off events.
        // 处理回退事件
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void backoffEventHandler(CSR_MESH_BACKOFF_EVENT_DATA_T *pEventData)
{
    switch(pEventData->event)
    {
        case CSR_MESH_BACKOFF_EVENT_DISABLE_BRIDGE:
        {
            /* Set backoff status bit */
            g_lightapp_data.backoff_status |= BRIDGE_BACKOFF_STATUS_MASK;
            DEBUG_STR("DISABLE BRIDGE\r\n");
        }
        break;

        case CSR_MESH_BACKOFF_EVENT_ENABLE_BRIDGE:
        {
            /* Reset backoff status bit */
            g_lightapp_data.backoff_status &= (~BRIDGE_BACKOFF_STATUS_MASK);
            DEBUG_STR("ENABLE BRIDGE\r\n");
        }
        break;

        case CSR_MESH_BACKOFF_EVENT_DISABLE_RELAY:
        {
            /* Set backoff status bit */
            g_lightapp_data.backoff_status |= RELAY_BACKOFF_STATUS_MASK;

            /* Stop relay if it was already enabled */
            if(g_lightapp_data.bearer_data.bearerRelayActive | BLE_BEARER_MASK)
            {
                CsrMeshRelayEnable(FALSE);
                DEBUG_STR("DISABLE RELAY\r\n");
            }
        }
        break;

        case CSR_MESH_BACKOFF_EVENT_ENABLE_RELAY:
        {
            /* Reset backoff status bit */
            g_lightapp_data.backoff_status &= (~RELAY_BACKOFF_STATUS_MASK);

            /* Start relay if it was already enabled */
            if(g_lightapp_data.bearer_data.bearerRelayActive | BLE_BEARER_MASK)
            {
                CsrMeshRelayEnable(TRUE);
                DEBUG_STR("ENABLE RELAY\r\n");
            }
        }
        break;

        default:
        break;
    }
}
#endif /* ENABLE_BACKOFF */

/*============================================================================*
 *  Public Function Definitions
 *============================================================================*/
#ifdef NVM_TYPE_FLASH
/*----------------------------------------------------------------------------*
 *  NAME
 *      WriteApplicationAndServiceDataToNVM
 *
 *  DESCRIPTION
 *      This function writes the application data to NVM. This function should
 *      be called on getting nvm_status_needs_erase
        // 该函数将应用数据写入NVM 中。当得到 nvm_status_needs_erase 信号时调用 
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
extern void WriteApplicationAndServiceDataToNVM(void)
{
    uint16 nvm_sanity = 0xffff;
    nvm_sanity = NVM_SANITY_MAGIC;

    /* Write NVM sanity word to the NVM */
    Nvm_Write(&nvm_sanity, sizeof(nvm_sanity), NVM_OFFSET_SANITY_WORD);

    /* Store the Association State */
    Nvm_Write((uint16 *)&g_lightapp_data.assoc_state,
             sizeof(g_lightapp_data.assoc_state),
              NVM_OFFSET_ASSOCIATION_STATE);

    /* Write GAP service data into NVM */
    WriteGapServiceDataInNVM();
}
#endif /* NVM_TYPE_FLASH */

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppSetState
 *
 *  DESCRIPTION
 *      This function is used to set the state of the application.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
extern void AppSetState(app_state new_state)
{
    /* Check if the new state to be set is not the same as the present state
     * of the application.
     */
     // 原来更改模块状态的函数是自己实现的，而不是自带的
    app_state old_state = g_lightapp_data.state;

    if (old_state != new_state)
    {
        /* Handle exiting old state */
        switch (old_state)
        {
            case app_state_init:
            break;

            case app_state_disconnecting:
                /* Common things to do whenever application exits
                 * app_state_disconnecting state.
                 */

                /* Initialise CSRmesh light data and services
                 * data structure while exiting Disconnecting state.
                 */

                smLightDataInit();
            break;

            case app_state_fast_advertising:
            case app_state_slow_advertising:
                /* Common things to do whenever application exits
                 * APP_*_ADVERTISING state.
                 */
                /* Stop on-going advertisements */
                GattStopAdverts();
                appAdvertisingExit();
            break;

            case app_state_connected:
                /* Do nothing here */
            break;

            case app_state_idle:
            {
            }
            break;

            default:
                /* Nothing to do */
            break;
        }

        /* Set new state */
        g_lightapp_data.state = new_state;

        /* Handle entering new state */
        switch (new_state)
        {
            case app_state_fast_advertising:
            {
                GattTriggerFastAdverts();
            }
            break;

            case app_state_slow_advertising:
            {
                /* Print an error message if we are in
                 * Slow Advertising state by mistake.
                 */
                DEBUG_STR("\r\nERROR: in SLOW ADVERTISING\r\n");
            }
            break;

            case app_state_idle:
            {
                CsrMeshStart();
            }
            break;

            case app_state_connected:
            {
                DEBUG_STR("Connected\r\n");
                /* Common things to do whenever application enters
                 * app_state_connected state.
                 */

                /* Stop on-going advertisements */
                GattStopAdverts();
            }
            break;

            case app_state_disconnecting:
                GattDisconnectReq(g_lightapp_data.gatt_data.st_ucid);
            break;

            default:
            break;
        }
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      ReportPanic
 *
 *  DESCRIPTION
 *      This function calls firmware panic routine and gives a single point
 *      of debugging any application level panics
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
extern void ReportPanic(app_panic_code panic_code)
{
    /* Raise panic */
    Panic(panic_code);
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      AppPowerOnReset
 *
 *  DESCRIPTION
 *      This user application function is called just after a power-on reset
 *      (including after a firmware panic), or after a wakeup from Hibernate or
 *      Dormant sleep states.
 *
 *      At the time this function is called, the last sleep state is not yet
 *      known.
 *
 *      NOTE: this function should only contain code to be executed after a
 *      power-on reset or panic. Code that should also be executed after an
 *      HCI_RESET should instead be placed in the AppInit() function.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
void AppPowerOnReset(void)
{
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      AppInit
 *
 *  DESCRIPTION
 *      This user application function is called after a power-on reset
 *      (including after a firmware panic), after a wakeup from Hibernate or
 *      Dormant sleep states, or after an HCI Reset has been requested.
 *
 *      The last sleep state is provided to the application in the parameter.
 *
 *      NOTE: In the case of a power-on reset, this function is called
 *      after app_power_on_reset().
 *

 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
void AppInit(sleep_state last_sleep_state)
{
    uint16 gatt_db_length = 0;
    uint16 *p_gatt_db_pointer = NULL;
    bool light_poweron = FALSE;

#ifdef USE_STATIC_RANDOM_ADDRESS
    /* Generate random address for the CSRmesh Device. */
    generateStaticRandomAddress(&g_lightapp_data.random_bd_addr);

    /* Set the Static Random Address of the device. */
    GapSetRandomAddress(&g_lightapp_data.random_bd_addr);
#endif /* USE_STATIC_RANDOM_ADDRESS */

    /* Initialise the application timers */
    TimerInit(MAX_APP_TIMERS, (void*)app_timers);
    g_lightapp_data.nvm_tid = TIMER_INVALID;

#ifdef DEBUG_ENABLE
    /* Initialize UART and configure with
     * default baud rate and port configuration.
     */
    DebugInit(UART_BUF_SIZE_BYTES_32, UartDataRxCallback, NULL);

    /* UART Rx threshold is set to 1,
     * so that every byte received will trigger the rx callback.
     */
    UartRead(1, 0);
#endif /* DEBUG_ENABLE */

    DEBUG_STR("\r\nLight Application\r\n");

    /* Initialise GATT entity */
    GattInit();

    /* Install GATT Server support for the optional Write procedure
     * This is mandatory only if control point characteristic is supported.
     */
    GattInstallServerWriteLongReliable();

    /* Don't wakeup on UART RX line */
    SleepWakeOnUartRX(FALSE);

#ifdef NVM_TYPE_EEPROM
    /* Configure the NVM manager to use I2C EEPROM for NVM store */
    NvmConfigureI2cEeprom();
#elif NVM_TYPE_FLASH
    /* Configure the NVM Manager to use SPI flash for NVM store. */
    NvmConfigureSpiFlash();
#endif /* NVM_TYPE_EEPROM */

    NvmDisable();
    /* Initialize the GATT and GAP data.
     * Needs to be done before readPersistentStore
     */
    smLightDataInit();

    /* Read persistent storage.
     * Call this before CsrMeshInit.
     */
     // q_cl:读到哪里去了？ -- 这个函数比较长，需要重新读一次
    readPersistentStore();

    /* Set the CsrMesh NVM start offset.
     * Note: This function must be called before the CsrMeshInit()
     */
     // q_cl: 这个 offset 能更改吗，还是固定的？
    CsrMeshNvmSetStartOffset(NVM_OFFSET_CSRMESH_LIB);

    /* Initialise the CSRmesh */
    CsrMeshInit();

    /* Enable Relay on Light */
    if (g_lightapp_data.bearer_data.bearerRelayActive & BLE_BEARER_MASK)
    {
        CsrMeshRelayEnable(TRUE);
    }

    /* Enable Notifications for raw messages */
    CsrMeshEnableRawMsgEvent(TRUE);

    /* Initialize the light model */
    LightModelInit(light_model_groups, MAX_MODEL_GROUPS);

    /* Initialize the power model */
    PowerModelInit(power_model_groups, MAX_MODEL_GROUPS);

    /* Initialize Bearer Model */
    BearerModelInit();

#ifdef ENABLE_FIRMWARE_MODEL
    /* Initialize Firmware Model */
    FirmwareModelInit();

    /* Set Firmware Version */
    g_lightapp_data.fw_version.major_version = APP_MAJOR_VERSION;
    g_lightapp_data.fw_version.minor_version = APP_MINOR_VERSION;
#endif /* ENABLE_FIRMWARE_MODEL */

    /* Initialize Attention Model */
    AttentionModelInit(attention_model_groups, MAX_MODEL_GROUPS);

#ifdef ENABLE_BATTERY_MODEL
    BatteryModelInit();
#endif /* ENABLE_BATTERY_MODEL */

#ifdef ENABLE_DATA_MODEL
    AppDataStreamInit(data_model_groups, MAX_MODEL_GROUPS);
#endif /* ENABLE_DATA_MODEL */

    /* Start CSRmesh */
    CsrMeshStart();

    /* Set the CSRmesh device UUID */
    CsrMeshSetDeviceUUID((CSR_MESH_UUID_T *)light_uuid);

#ifdef USE_AUTHORIZATION_CODE
    /* Set the CSRmesh device Authorization Code */
    CsrMeshSetAuthorisationCode((CSR_MESH_AUTH_CODE_T *)light_auth_code);
#endif /* USE_AUTHORIZATION_CODE */

    /* Tell Security Manager module about the value it needs to initialize it's
     * diversifier to.
     */
    SMInit(0);

    /* Initialise CSRmesh light application State */
    g_lightapp_data.state = app_state_init;

    /* Initialize Light Hardware */
    // 重点来了，灯光硬件初始化 -- 为自带函数？
    LightHardwareInit();

#ifdef USE_ASSOCIATION_REMOVAL_KEY
    IOTSwitchInit();
#endif /* USE_ASSOCIATION_REMOVAL_KEY */

    /* Start a timer which does device ID adverts till the time device
     * is associated
     */
     // 启动deviceID广播，只要设备被绑定了
     // 下面这个判断逻辑有点搞，容易迷惑。两个同为真的含义不同
    if(app_state_not_associated == g_lightapp_data.assoc_state)
    {
        smAppInitiateAssociation();
    }
    else
    {
        DEBUG_STR("Light is associated\r\n");

        /* Light is already associated. Set the colour from NVM */
        LightHardwareSetColor(g_lightapp_data.light_state.red,
                              g_lightapp_data.light_state.green,
                              g_lightapp_data.light_state.blue);


        /* Set the light power as read from NVM */
        if ((g_lightapp_data.power.power_state == POWER_STATE_ON) ||
            (g_lightapp_data.power.power_state == POWER_STATE_ON_FROM_STANDBY))
        {
            light_poweron = TRUE;
        }
        
        LightHardwarePowerControl(light_poweron);
    }

    /* Tell GATT about our database. We will get a GATT_ADD_DB_CFM event when
     * this has completed.
     */
     // 告诉GATT关于模块的数据库，完成后，会得到 GATT_ADD_DB_CFM 事件
    p_gatt_db_pointer = GattGetDatabase(&gatt_db_length);
    GattAddDatabaseReq(gatt_db_length, p_gatt_db_pointer);

#ifdef ENABLE_BACKOFF
    if( g_lightapp_data.bearer_data.bearerEnabled & BLE_GATT_SERVER_BEARER_MASK)
    {
        /* Enable bridge backoff only if GATT server bearer is enabled. */
        CsrMeshEnableBackoff(CSR_MESH_BACKOFF_BRIDGE, TRUE);
    }
    if (g_lightapp_data.bearer_data.bearerRelayActive & BLE_BEARER_MASK)
    {
        /* Enable relay backoff only if relay is enabled. */
        CsrMeshEnableBackoff(CSR_MESH_BACKOFF_RELAY, TRUE);
    }
#endif /* ENABLE_BACKOFF */
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      AppProcesSystemEvent
 *
 *  DESCRIPTION
 *      This user application function is called whenever a system event, such
 *      as a battery low notification, is received by the system.
        // 当任何一个系统事件被系统接受到，这个应用程序都会被调用 。
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
void AppProcessSystemEvent(sys_event_id id, void *data)
{
    switch (id)
    {
        case sys_event_pio_changed:
        {
#ifdef USE_ASSOCIATION_REMOVAL_KEY
            handlePIOEvent((pio_changed_data*)data);
#endif
        }
        break;

        default:
        break;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppProcessLmEvent
 *
 *  DESCRIPTION
 *      This user application function is called whenever a LM-specific event
 *      is received by the system.
        // 当系统接受到 LM-specific 事件时，该函数被调用 。即层管理事件
 *
 * PARAMETERS
 *      event_code [in]   LM event ID
 *      event_data [in]   LM event data
 *
 * RETURNS
 *      TRUE if the application has finished with the event data;
 *           the control layer will free the buffer.
        // 当程序完成事件数据，返回真；同时层控制会释放buffer。
 *----------------------------------------------------------------------------*/
extern bool AppProcessLmEvent(lm_event_code event_code,
                              LM_EVENT_T *p_event_data)
{
    switch(event_code)
    {
        /* Handle events received from Firmware */

        case GATT_ADD_DB_CFM:
            /* Attribute database registration confirmation */
            /* 属性数据库注册确认 */
            handleSignalGattAddDBCfm((GATT_ADD_DB_CFM_T*)p_event_data);
        break;

        case GATT_CANCEL_CONNECT_CFM:
            /* Confirmation for the completion of GattCancelConnectReq()
             * procedure
             */
             // 确定 GattCancelConnectReq() 程序执行完成
            handleSignalGattCancelConnectCfm();
        break;

        case LM_EV_CONNECTION_COMPLETE:
            /* Handle the LM connection complete event. */
            handleSignalLmEvConnectionComplete((LM_EV_CONNECTION_COMPLETE_T*)
                                                                p_event_data);
        break;

        case GATT_CONNECT_CFM:
            /* Confirmation for the completion of GattConnectReq()
             * procedure
             */
            handleSignalGattConnectCfm((GATT_CONNECT_CFM_T*)p_event_data);
        break;

        case SM_SIMPLE_PAIRING_COMPLETE_IND:
            /* Indication for completion of Pairing procedure */
            handleSignalSmSimplePairingCompleteInd(
                (SM_SIMPLE_PAIRING_COMPLETE_IND_T*)p_event_data);
        break;

        case LM_EV_ENCRYPTION_CHANGE:
            /* Indication for encryption change event */
            /* Nothing to do */
        break;

        /* Received in response to the LsConnectionParamUpdateReq()
         * request sent from the slave after encryption is enabled. If
         * the request has failed, the device should again send the same
         * request only after Tgap(conn_param_timeout). Refer Bluetooth 4.0
         * spec Vol 3 Part C, Section 9.3.9 and HID over GATT profile spec
         * section 5.1.2.
         */
         // lm 交换？
         // 当应答从设备允许加密后发送的 LsConnectionParamUpdataeReq() 信号后接收
         // 如果请求失败，设备重新发送请求。
        case LS_CONNECTION_PARAM_UPDATE_CFM:
            handleSignalLsConnParamUpdateCfm(
                (LS_CONNECTION_PARAM_UPDATE_CFM_T*) p_event_data);
        break;

        case LM_EV_CONNECTION_UPDATE:
            /* This event is sent by the controller on connection parameter
             * update.
             */
            handleSignalLmConnectionUpdate(
                            (LM_EV_CONNECTION_UPDATE_T*)p_event_data);
        break;

        case LS_CONNECTION_PARAM_UPDATE_IND:
            /* Indicates completion of remotely triggered Connection
             * parameter update procedure
             */
            handleSignalLsConnParamUpdateInd(
                            (LS_CONNECTION_PARAM_UPDATE_IND_T *)p_event_data);
        break;

        case GATT_ACCESS_IND:
            /* Indicates that an attribute controlled directly by the
             * application (ATT_ATTR_IRQ attribute flag is set) is being
             * read from or written to.
             */
            handleSignalGattAccessInd((GATT_ACCESS_IND_T*)p_event_data);
        break;

        case GATT_DISCONNECT_IND:
            /* Disconnect procedure triggered by remote host or due to
             * link loss is considered complete on reception of
             * LM_EV_DISCONNECT_COMPLETE event. So, it gets handled on
             * reception of LM_EV_DISCONNECT_COMPLETE event.
             */
         break;

        case GATT_DISCONNECT_CFM:
            /* Confirmation for the completion of GattDisconnectReq()
             * procedure is ignored as the procedure is considered complete
             * on reception of LM_EV_DISCONNECT_COMPLETE event. So, it gets
             * handled on reception of LM_EV_DISCONNECT_COMPLETE event.
             */
        break;

        case LM_EV_DISCONNECT_COMPLETE:
        {
            /* Disconnect procedures either triggered by application or remote
             * host or link loss case are considered completed on reception
             * of LM_EV_DISCONNECT_COMPLETE event
             */
             handleSignalLmDisconnectComplete(
                    &((LM_EV_DISCONNECT_COMPLETE_T *)p_event_data)->data);
        }
        break;

        case LM_EV_ADVERTISING_REPORT:
        {
            CsrMeshProcessMessage((LM_EV_ADVERTISING_REPORT_T *)p_event_data);
        }
        break;

        case LS_RADIO_EVENT_IND:
        {
            CsrMeshHandleRadioEvent();
        }
        break;

        default:
            /* Ignore any other event */
        break;

    }

    return TRUE;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppProcessCsrMeshEvent
 *
 *  DESCRIPTION
 *      This user application function is called whenever a CSRmesh event
 *      is received by the system.
        // 这里应该是主角，用来处理系统接受到 CSRmesh 事件
 *
 * PARAMETERS
 *      event_code csr_mesh_event_t
 *      data       Data associated with the event
 *      length     Length of the data
 *      state_data Pointer to the variable pointing to state data.
 *
 * RETURNS
 *      TRUE if the app has finished with the event data; the control layer
 *      will free the buffer.
        // 当程序处理完事件数据，返回为真。同时控制层将释放 buffer 
 *----------------------------------------------------------------------------*/
extern void AppProcessCsrMeshEvent(csr_mesh_event_t event_code, uint8* data,
                                   uint16 length, void **state_data)
{
    bool start_nvm_timer = FALSE;
    bool update_lastetag = FALSE;

    switch(event_code)
    {
        case CSR_MESH_ASSOCIATION_REQUEST:
        {
            if( g_lightapp_data.assoc_state != app_state_association_started)
            {
                g_lightapp_data.assoc_state = app_state_association_started;
            }
            TimerDelete(g_lightapp_data.mesh_device_id_advert_tid);
            g_lightapp_data.mesh_device_id_advert_tid = TIMER_INVALID;

            /* Blink Light in Yellow to indicate association started */
            LightHardwareSetBlink(127, 127, 0, 32, 32);
        }
        break;

        case CSR_MESH_KEY_DISTRIBUTION:
        {
            DEBUG_STR("Association complete\r\n");
            g_lightapp_data.assoc_state = app_state_associated;

            /* Write association state to NVM */
            Nvm_Write((uint16 *)&g_lightapp_data.assoc_state,
                     sizeof(g_lightapp_data.assoc_state),
                     NVM_OFFSET_ASSOCIATION_STATE);

            /* Restore light settings after association */
            LightHardwareSetColor(g_lightapp_data.light_state.red,
                                  g_lightapp_data.light_state.green,
                                  g_lightapp_data.light_state.blue);

            /* Restore power settings after association */
            LightHardwarePowerControl(g_lightapp_data.power.power_state);
        }
        break;

        case CSR_MESH_CONFIG_RESET_DEVICE:
        {
            DEBUG_STR("Reset Device\r\n");

            /* Move device to dissociated state */
            g_lightapp_data.assoc_state = app_state_not_associated;

            /* Write association state to NVM */
            Nvm_Write((uint16 *)&g_lightapp_data.assoc_state,
                     sizeof(g_lightapp_data.assoc_state),
                     NVM_OFFSET_ASSOCIATION_STATE);

            /* Reset the supported model groups and save it to NVM */
            /* Light model */
            MemSet(light_model_groups, 0x0000, sizeof(light_model_groups));
            Nvm_Write((uint16 *)light_model_groups, sizeof(light_model_groups),
                                                 NVM_OFFSET_LIGHT_MODEL_GROUPS);

            /* Power model */
            MemSet(power_model_groups, 0x0000, sizeof(power_model_groups));
            Nvm_Write((uint16 *)power_model_groups, sizeof(power_model_groups),
                                                 NVM_OFFSET_POWER_MODEL_GROUPS);

            /* Attention model */
            MemSet(attention_model_groups, 0x0000, sizeof(attention_model_groups));
            Nvm_Write((uint16 *)attention_model_groups, sizeof(attention_model_groups),
                                            NVM_OFFSET_ATTENTION_MODEL_GROUPS);

#ifdef ENABLE_DATA_MODEL
            /* Data stream model */
            MemSet(data_model_groups, 0x0000, sizeof(data_model_groups));
            Nvm_Write((uint16 *)data_model_groups, sizeof(data_model_groups),
                                            NVM_OFFSET_DATA_MODEL_GROUPS);
#endif /* ENABLE_DATA_MODEL */

            /* Reset Light State */
            // 重置灯泡的颜色、亮度。可以从这里进行修改？
            g_lightapp_data.light_state.red   = 0xFF;
            g_lightapp_data.light_state.green = 0xFF;
            g_lightapp_data.light_state.blue  = 0xFF;
            g_lightapp_data.light_state.power = POWER_STATE_OFF;
            g_lightapp_data.power.power_state = POWER_STATE_OFF;
            start_nvm_timer = TRUE;

            /* Start Mesh association again */
            smAppInitiateAssociation();
        }
        break;

        case CSR_MESH_CONFIG_DEVICE_IDENTIFIER:
        {
            DEBUG_STR("Device ID received:");
            // 设备接受到的分配给自己的 ID ，data 的哪个才是 ID 
            DEBUG_U8(data[0]);
            DEBUG_U8(data[1]);
            DEBUG_STR("\r\n");
        }
        break;

        case CSR_MESH_CONFIG_GET_VID_PID_VERSION:
        {
            if (state_data != NULL)
            {
                *state_data = (void *)&vid_pid_info;
            }
        }
        break;

        case CSR_MESH_CONFIG_GET_APPEARANCE:
        {
            if (state_data != NULL)
            {
                *state_data = (void *)&device_appearance;
            } 
        }
        break;

        case CSR_MESH_GROUP_SET_MODEL_GROUPID:
        {
            update_lastetag = handleCsrMeshGroupSetMsg(data, length);
        }
        break;

        case CSR_MESH_LIGHT_SET_LEVEL:
        {
            /* Update State of RGB in application */
            // 更新程序中 RGB 的状态
            g_lightapp_data.light_state.level = data[0];
            g_lightapp_data.light_state.power = POWER_STATE_ON;
            g_lightapp_data.power.power_state = POWER_STATE_ON;
            start_nvm_timer = TRUE;

            /* Set the White light level */
            LightHardwareSetLevel(g_lightapp_data.light_state.red, 
                                  g_lightapp_data.light_state.green,
                                  g_lightapp_data.light_state.blue,
                                  g_lightapp_data.light_state.level);

            /* Send Light State Information to Model */
            // 发送灯泡信息给模型？？？？ q_cl: 
            if (state_data != NULL)
            {
                *state_data = (void *)&g_lightapp_data.light_state;
            }

            /* Don't apply to hardware unless light is ON */
            DEBUG_STR("Set Level: ");
            DEBUG_U8(data[0]);
            DEBUG_STR("\r\n");
        }
        break;

        case CSR_MESH_LIGHT_SET_RGB:
        {
            /* Update State of RGB in application */
            g_lightapp_data.light_state.red   = data[1];
            g_lightapp_data.light_state.green = data[2];
            g_lightapp_data.light_state.blue  = data[3];
            g_lightapp_data.light_state.power = POWER_STATE_ON;
            g_lightapp_data.power.power_state = POWER_STATE_ON;
            start_nvm_timer = TRUE;

            /* Set Colour of light */
            LightHardwareSetColor(g_lightapp_data.light_state.red,
                                  g_lightapp_data.light_state.green,
                                  g_lightapp_data.light_state.blue);

            /* Send Light State Information to Model */
            if (state_data != NULL)
            {
                *state_data = (void *)&g_lightapp_data.light_state;
            }

            DEBUG_STR("Set RGB : ");
            DEBUG_U8(data[1]);
            DEBUG_STR(",");
            DEBUG_U8(data[2]);
            DEBUG_STR(",");
            DEBUG_U8(data[3]);
            DEBUG_STR("\r\n");
        }
        break;

        case CSR_MESH_LIGHT_SET_COLOR_TEMP:
        {
#ifdef COLOUR_TEMP_ENABLED
            uint16 temp = 0;
            temp = ((uint16)data[1] << 8) | (data[0]);

            g_lightapp_data.power.power_state = POWER_STATE_ON;
            g_lightapp_data.light_state.power = POWER_STATE_ON;

            /* Set Colour temperature of light */
            LightHardwareSetColorTemp(temp);

            /* Send Light State Information to Model */
            if (state_data != NULL)
            {
                *state_data = (void *)&g_lightapp_data.light_state;
            }

            DEBUG_STR("Set Colour Temperature: ");
            DEBUG_U16(temp);
            DEBUG_STR("\r\n");
#endif /* COLOUR_TEMP_ENABLED */
        }
        break;

        case CSR_MESH_LIGHT_GET_STATE:
        {
            /* Send Light State Information to Model */
            if (state_data != NULL)
            {
                *state_data = (void *)&g_lightapp_data.light_state;
            }
        }
        break;

        case CSR_MESH_POWER_GET_STATE:
        {
            /* Send Power State Information to Model */
            if (state_data != NULL)
            {
                *state_data = (void *)&g_lightapp_data.power;
            }
        }
        break;

        case CSR_MESH_POWER_TOGGLE_STATE:
        case CSR_MESH_POWER_SET_STATE:
        {
            if (CSR_MESH_POWER_SET_STATE == event_code)
            {
                g_lightapp_data.power.power_state = data[0];
            }
            else
            {
                togglePowerState();
                // toggle 切换；切换灯泡状态。而非只是更改成指定的状态
            }

            DEBUG_STR("Set Power: ");
            DEBUG_U8(g_lightapp_data.power.power_state);
            DEBUG_STR("\r\n");
            // 应用中的开关被限制或者灯泡处于关闭状态，不能更改灯光的亮度？
            if (g_lightapp_data.power.power_state == POWER_STATE_OFF ||
                g_lightapp_data.power.power_state == POWER_STATE_STANDBY)
            {
                LightHardwarePowerControl(FALSE);
            }
            else if(g_lightapp_data.power.power_state == POWER_STATE_ON ||
                    g_lightapp_data.power.power_state == \
                                                POWER_STATE_ON_FROM_STANDBY)
            {
                LightHardwareSetColor(g_lightapp_data.light_state.red,
                                      g_lightapp_data.light_state.green,
                                      g_lightapp_data.light_state.blue);

                /* Turn on with old colour value restored */
                LightHardwarePowerControl(TRUE);
            }

            g_lightapp_data.light_state.power =
                                            g_lightapp_data.power.power_state;

            /* Send Power State Information to Model */
            if (state_data != NULL)
            {
                *state_data = (void *)&g_lightapp_data.power;
            }

            start_nvm_timer = TRUE;
        }
        break;

        case CSR_MESH_BEARER_SET_STATE:
        {
            uint8 *pData = data;
            bool b_enable;
            g_lightapp_data.bearer_data.bearerRelayActive = BufReadUint16(&pData);
            g_lightapp_data.bearer_data.bearerEnabled     = BufReadUint16(&pData);

            /* BLE Advert Bearer is always enabled on this device. */
            // BLE 广播载体在这个设备上总是被允许
            g_lightapp_data.bearer_data.bearerEnabled    |= BLE_BEARER_MASK;

            /* Update BLE Bearer(adverts) relay active status */
            // 更新BLE 传播中继的活动状态
            b_enable = (g_lightapp_data.bearer_data.bearerRelayActive & \
                                                 BLE_BEARER_MASK)? TRUE : FALSE;
            CsrMeshRelayEnable(b_enable);

#ifdef ENABLE_BACKOFF
            CsrMeshEnableBackoff(CSR_MESH_BACKOFF_RELAY, b_enable);

            /* Enable bridge backoff if GATT Server bearer is enabled */
            b_enable = (g_lightapp_data.bearer_data.bearerEnabled & \
                                     BLE_GATT_SERVER_BEARER_MASK)? TRUE : FALSE;

            CsrMeshEnableBackoff(CSR_MESH_BACKOFF_BRIDGE, b_enable);
#endif /* ENABLE_BACKOFF */

            /* Update Bearer Model Data to NVM */
            Nvm_Write((uint16 *)&g_lightapp_data.bearer_data,
                      sizeof(BEARER_MODEL_STATE_DATA_T), NVM_BEARER_DATA_OFFSET);

            /* Update LastETag. */
            update_lastetag = TRUE;
        }
        /* Fall through */
        case CSR_MESH_BEARER_GET_STATE:
        {
            if (state_data != NULL)
            {
                *state_data = (void *)&g_lightapp_data.bearer_data;
            }
        }
        break;

#ifdef ENABLE_FIRMWARE_MODEL
        case CSR_MESH_FIRMWARE_GET_VERSION_INFO:
        {
            /* Send Firmware Version data to the model */
            if (state_data != NULL)
            {
                *state_data = (void *)&g_lightapp_data.fw_version;
            }
        }
        break;

        case CSR_MESH_FIRMWARE_UPDATE_REQUIRED:
        {
            BD_ADDR_T *pBDAddr = NULL;
#ifdef USE_STATIC_RANDOM_ADDRESS
            pBDAddr = &g_lightapp_data.random_bd_addr;
#endif /* USE_STATIC_RANDOM_ADDRESS */

            DEBUG_STR("\r\n FIRMWARE UPDATE IN PROGRESS \r\n");

            /* Write the value CSR_OTA_BOOT_LOADER to NVM so that
             * it starts in OTA mode upon reset
             */
            OtaWriteCurrentApp(csr_ota_boot_loader,
                               FALSE,   /* is bonded */
                               NULL,    /* Typed host BD Address */
                               0,       /* Diversifier */
                               pBDAddr, /* local_random_address */
                               NULL,    /* irk */
                               FALSE    /* service_changed_config */
                              );

           /* Defer OTA Reset for half a second to ensure that,
            * acknowledgements are sent before reset.
            */
           ota_rst_tid = TimerCreate(OTA_RESET_DEFER_DURATION, TRUE,
                                     issueOTAReset);

           /* Update LastETag. */
           update_lastetag = TRUE;
        }
        break;
#endif /* ENABLE_FIRMWARE_MODEL */

#ifdef ENABLE_BATTERY_MODEL
        case CSR_MESH_BATTERY_GET_STATE:
        {
            /* Read Battery Level */
            g_lightapp_data.battery_data.battery_level = ReadBatteryLevel();

            /* Pass Battery state data to model */
            if (state_data != NULL)
            {
                *state_data = (void *)&g_lightapp_data.battery_data;
            }
        }
        break;
#endif /* ENABLE_BATTERY_MODEL */

        case CSR_MESH_ATTENTION_SET_STATE:
        {
            uint32 dur_us;
            /* Read the data */
            g_lightapp_data.attn_data.attract_attn  = BufReadUint8(&data);
            g_lightapp_data.attn_data.attn_duration = BufReadUint16(&data);

            /* Delete attention timer if it exists */
            if (TIMER_INVALID != attn_tid)
            {
                TimerDelete(attn_tid);
                attn_tid = TIMER_INVALID;
            }

            /* If attention Enabled */
            if (g_lightapp_data.attn_data.attract_attn)
            {
                /* Create attention duration timer if required */
                if (g_lightapp_data.attn_data.attn_duration != 0xFFFF)
                {
                    dur_us = (uint32)g_lightapp_data.attn_data.attn_duration * \
                                     MILLISECOND;
                    attn_tid = TimerCreate(dur_us, TRUE, attnTimerHandler);
                }

                /* Enable Red light blinking to attract attention */
                LightHardwareSetBlink(127, 0, 0, 32, 32);
            }
            else
            {
                /* Restore Light State */
                LightHardwareSetColor(g_lightapp_data.light_state.red,
                                      g_lightapp_data.light_state.green,
                                      g_lightapp_data.light_state.blue);

                /* Restore the light Power State */
                LightHardwarePowerControl(g_lightapp_data.power.power_state);
            }

            /* Send response data to model */
            if (state_data != NULL)
            {
                *state_data = (void *)&g_lightapp_data.attn_data;
            }

            /* Debug logs */
            DEBUG_STR("\r\n ATTENTION_SET_STATE : Enable :");
            DEBUG_U8(g_lightapp_data.attn_data.attract_attn);
            DEBUG_STR("Duration : ");
            DEBUG_U16(g_lightapp_data.attn_data.attn_duration);
            DEBUG_STR("\r\n");

        }
        break;

#ifdef ENABLE_DATA_MODEL
        /* Data stream model messages */
        case CSR_MESH_DATA_STREAM_SEND_CFM:
        {
            handleCSRmeshDataStreamSendCfm((CSR_MESH_STREAM_EVENT_T *)data);
        }
        break;

        case CSR_MESH_DATA_STREAM_DATA_IND:
        {
            handleCSRmeshDataStreamDataInd((CSR_MESH_STREAM_EVENT_T *)data);
        }
        break;

        /* Stream flush indication */
        case CSR_MESH_DATA_STREAM_FLUSH_IND:
        {
            handleCSRmeshDataStreamFlushInd((CSR_MESH_STREAM_EVENT_T *)data);
        }
        break;

        /* Received a single block of data */
        case CSR_MESH_DATA_BLOCK_IND:
        {
            handleCSRmeshDataBlockInd((CSR_MESH_STREAM_EVENT_T *)data);
        }
        break;
#endif /* ENABLE_DATA_MODEL */

#ifdef ENABLE_BACKOFF
        case CSR_MESH_BACKOFF_EVENT:
        {
            CSR_MESH_BACKOFF_EVENT_DATA_T *pBackoffData = \
                                           (CSR_MESH_BACKOFF_EVENT_DATA_T *)data;
            if (pBackoffData)
            {
                backoffEventHandler(pBackoffData);
            }
        }
        break;
#endif /* ENABLE_BACKOFF */

        /* Received a raw message from lower-layers.
         * Notify to the control device if connected.
         */
        case CSR_MESH_RAW_MESSAGE:
        {
            if (g_lightapp_data.state == app_state_connected)
            { 
                MeshControlNotifyResponse(g_lightapp_data.gatt_data.st_ucid,
                                          data, length);
            }
        }
        break;

        default:
        break;
    }

    /* Commit Update LastETag. */
    if (update_lastetag)
    {
        CsrMeshUpdateLastETag();
    }

    /* Start NVM timer if required */
    if (TRUE == start_nvm_timer)
    {
        /* Delete existing timer */
        if (TIMER_INVALID != g_lightapp_data.nvm_tid)
        {
            TimerDelete(g_lightapp_data.nvm_tid);
        }

        /* Re-start the timer */
        g_lightapp_data.nvm_tid = TimerCreate(NVM_WRITE_DEFER_DURATION,
                                              TRUE,
                                              smLightDataNVMWriteTimerHandler);
    }
}

