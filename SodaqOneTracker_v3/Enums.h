#ifndef _ENUMS_h
#define _ENUMS_h

enum LoraInitResult {
    LORA_INIT_FAIL = 0,
    LORA_INIT_FULL,
    LORA_INIT_DEVICE_ONLY
};

enum LoraInitConsoleMessages {
    LORA_INIT_SKIP_CONSOLE_MESSAGES,
    LORA_INIT_SHOW_CONSOLE_MESSAGES
};

enum LoraInitJoin {
    LORA_INIT_SKIP_JOIN,
    LORA_INIT_JOIN
};

#endif
