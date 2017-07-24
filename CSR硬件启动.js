1。AppProcessLmEvent() -- 系统接受到链接事件时触发
	Database Access:
	LS Events:
	LM Events:
		LM_EV_ADVERTISING_REPORT : 当接收到广播数据包时，接收该事件。应用程序将event data 传入 CsrMeshProcessMessage() 中进行处理数据包
		LM_EV_CONNECTION_COMPLETE: 当与主设备的连接，被视为完成且包含新的连接参数时，该事件发送出来
		LM_EV_DISCONNECT_COMPLETE:
		LM_EV_ENCRYPTION_CHANGE  : 链接加密发生更改
		LM_EV_CONNECTION_UPDATE  : 
	SMP Events:
		SM_SIMPLE_PAIRING_COMPLETE_IND: 表示配对完成
	GATT Events:
		GATT_CONNECT_CFM:
		GATT_CANCEL_CONNECT_CFM:
2.AppProcessSystemEvent() -- 系统事件，目前只有一个PIO口改变事件
	sys_event_pio_changed:
3.AppProcessCsrMeshEvent() -- CSRmesh network 事件，如改变灯光
	Raw Messages：
		CSR_MESH_RAW_MESSAGE : 
	Light Control Messages:
		CSR_MESH_LIGHT_SET_RGB :
		CSR_MESH_LIGHT_SET_LEVEL:
		CSR_MESH_LIGHT_SET_COLOR_TEMP:
	Power State Messages :
		CSR_MESH_POWER_TOGGLE_STATE:
		CSR_MESH_POWER_SET_STATE: 
	Network Association Messages :
		CSR_MESH_ASSOCIATION_REQUEST: 收到手机发送过来的绑定请求;应用开始闪烁表明正在绑定过程中，并停止广播设备UUID
		CSR_MESH_KEY_DISTRIBUTION:
	Device configuration Messages:
		CSR_MESH_CONFIG_DEVICE_IDENTIFIER:
		CSR_MESH_CONFIG_RESET_DEVICE:
		CSR_MESH_CONFIG_GET_VID_PID_VERSION:
		CSR_MESH_CONFIG_GET_APPEARANCE: 收到配置设备请求表征信息时;应用将这个请求信息发送至传输(发送信息)的库函数
	Firmware Model Messages:
		CSR_MESH_FIRMWARE_UPDATE_REQUIRED:
		CSR_MESH_FIRMWARE_GET_VERSION_INFO:
	Battery Model Messages:
		CSR_MESH_BATTERY_GET_STATE:
	Attention Model Messages:
		CSR_MESH_ATTENTION_SET_STATE:
	Group Model Messages:
		CSR_MESH_GROUP_SET_MODEL_GROUPID: 
	Bearer Model Messages:
		CSR_MESH_BEARER_SET_STATE:
		CSR_MESH_BEARER_GET_STATE:
	Stream Model Messages:
		CSR_MESH_DATA_STREAM_FLUSH_IND: 
		CSR_MESH_DATA_STREAM_DATA_IND:
		CSR_MESH_DATA_STREAM_SEND_CFM:
		CSR_MESH_DATA_BLOCK_IND: 
	Back-off Event Messages:
		CSR_MESH_BACKOFF_EVENT:
4.AppInit() -- 设备初始化操作
	