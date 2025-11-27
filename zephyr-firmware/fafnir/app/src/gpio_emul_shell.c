#include "gpio_emul_shell.h"
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(shell_gpio_emul);

#define OUTPUT_PIN_NODE DT_ALIAS(solenoids)
#define GPIO0 DEVICE_DT_GET(DT_ALIAS(gpio0))

struct output_pin_info {
    bool is_valid; 
    const char *label;
    struct gpio_dt_spec spec;
};

#define GET_OUTPUT_PIN_INFO(node_id) \
    COND_CODE_1(DT_NODE_HAS_PROP(node_id, gpios), \
    ( \
        { \
            .is_valid = true, \
            .label = DT_PROP(node_id, label), \
            .spec = GPIO_DT_SPEC_GET(node_id, gpios), \
        } \
    ), \
    ( \
        { .is_valid = false } /* Dummy entry for gpio_emul */ \
    ) \
    )

static const struct output_pin_info output_pin_list[] = {
    DT_FOREACH_CHILD_SEP(OUTPUT_PIN_NODE, GET_OUTPUT_PIN_INFO, (,))
};

static int cmd_display_emul_gpio_pins(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(sh, "--- Emulation Status ---");
    shell_print(sh, "%-15s | %-10s | %-5s | %s", "Label", "GPIO Dev", "Pin", "Level");
    shell_print(sh, "--------------------------------------------------");

    for (size_t i = 0; i < ARRAY_SIZE(output_pin_list); i++) {
        const struct output_pin_info *sol = &output_pin_list[i];

        if (!sol->is_valid) {
            continue;
        }

        if (!device_is_ready(sol->spec.port)) {
            shell_error(sh, "Error: Device %s is not ready", sol->spec.port->name);
            continue;
        }

        int state = gpio_emul_output_get(sol->spec.port, sol->spec.pin);

        shell_print(sh, "%-15s | %-10s | %-5d | %s", 
                    sol->label, 
                    sol->spec.port->name, 
                    sol->spec.pin, 
                    (state ? "HIGH (1)" : "LOW (0)"));
    }

    return 0;
}

SHELL_CMD_REGISTER(getsol, NULL, "Display output pin emulation states", &cmd_display_emul_gpio_pins);


static int cmd_set_emul_gpio_pins(const struct shell *sh, size_t argc, char **argv) {
    shell_print(sh, "letsgoo");
    gpio_emul_input_set(GPIO0, atoi(argv[1]), atoi(argv[2]));
    return 0;
}

// SHELL_CMD_REGISTER(setsol, NULL, "bla", &cmd_display_emul_gpio_pins);
SHELL_CMD_ARG_REGISTER(setsol, NULL, "bla", &cmd_set_emul_gpio_pins, 3, 0);