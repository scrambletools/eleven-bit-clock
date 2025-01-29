


sysTaskNotifyValue = static_cast<SYS_NOTIFY>(ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)));
SYS_NOTIFY sysTaskNotifyValue = (SYS_NOTIFY)0;

// define a notification for the ArkA task
enum class ARKA_NOTIFY : uint32_t 
{
    CMD_INC_BRIGHTNESS = 1,
    CMD_DEC_BRIGHTNESS,
    CMD_ON,
    CMD_OFF,
    CMD_SHUT_DOWN,
};

// to send a notification to a task
if (taskHandleAWSRun != nullptr) // Inform AWS that it may NOT send Internet messages
                        while (!xTaskNotify(taskHandleAWSRun, static_cast<uint32_t>(AWS_NOTIFY::NFY_INET_NOT_READY), eSetValueWithoutOverwrite))
                            vTaskDelay(pdMS_TO_TICKS(10));

