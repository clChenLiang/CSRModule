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
		CSR_MESH_CONFIG_RESET_DEVICE: 当配置设备（手机？）想移除该设备上所有的的mesh组网信息时接受到该信号。应用将重置所有的组ID并重新广播UUID
		CSR_MESH_CONFIG_GET_VID_PID_VERSION:
		CSR_MESH_CONFIG_GET_APPEARANCE: 收到配置设备请求表征信息时;应用将这个请求信息发送至传输(发送信息)的库函数
	Firmware Model Messages:
		CSR_MESH_FIRMWARE_UPDATE_REQUIRED: 当控制设备接受到固件升级请求时，收到该信息。应用将会切换为 OTAU bootloader 模式
		CSR_MESH_FIRMWARE_GET_VERSION_INFO: 配置设备请求设备外观信息的时候，收到该信号
	Battery Model Messages:
		CSR_MESH_BATTERY_GET_STATE:
	Attention Model Messages:
		CSR_MESH_ATTENTION_SET_STATE:
	Group Model Messages:
		CSR_MESH_GROUP_SET_MODEL_GROUPID: 控制设备设置一个新的组ID时接受到该信号，
	Bearer Model Messages:
		CSR_MESH_BEARER_SET_STATE:
		CSR_MESH_BEARER_GET_STATE:
	Stream Model Messages:
		CSR_MESH_DATA_STREAM_FLUSH_IND: 数据流客户端想开始发送数据流或者表明当前数据流数据传送结束时接受到该信号
		CSR_MESH_DATA_STREAM_DATA_IND: 当接受到数据流客户端发送的下一个数据块时，接受到该信号。应用会验证传送的数据流的格式，并存到
		CSR_MESH_DATA_STREAM_SEND_CFM:
		CSR_MESH_DATA_BLOCK_IND: 
	Back-off Event Messages:
		CSR_MESH_BACKOFF_EVENT:
4.AppInit() -- 设备初始化操作
