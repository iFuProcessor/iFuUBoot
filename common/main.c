// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2000
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 */

/* #define	DEBUG	*/

#include <common.h>
#include <autoboot.h>
#include <cli.h>
#include <console.h>
#include <version.h>

/*
 * Board-specific Platform code can reimplement show_boot_progress () if needed
 */
__weak void show_boot_progress(int val) {}

static void run_preboot_environment_command(void)
{
#ifdef CONFIG_PREBOOT
	char *p;

	p = env_get("preboot");
	if (p != NULL) {
		int prev = 0;

		if (IS_ENABLED(CONFIG_AUTOBOOT_KEYED))
			prev = disable_ctrlc(1); /* disable Ctrl-C checking */

		run_command_list(p, -1, 0);

		if (IS_ENABLED(CONFIG_AUTOBOOT_KEYED))
			disable_ctrlc(prev);	/* restore Ctrl-C checking */
	}
#endif /* CONFIG_PREBOOT */
}

static const char* logo_1 =
"    ______            __                 __  __        _                           _  __        \n\
   / ____/__  __ ____/ /____ _ ____     / / / /____   (_)_   __ ___   _____ _____ (_)/ /_ __  __ \n\
  / /_   / / / // __  // __ `// __ \\   / / / // __ \\ / /| | / // _ \\ / ___// ___// // __// / / / \n\
 / __/  / /_/ // /_/ // /_/ // / / /  / /_/ // / / // / | |/ //  __// /   (__  )/ // /_ / /_/ /  \n\
/_/     \\__,_/ \\__,_/ \\__,_//_/ /_/   \\____//_/ /_//_/  |___/ \\___//_/   /____//_/ \\__/ \\__, /   \n\
                                                                                       /____/    \n";
static const char* logo_2 =
"                                                          _  ______        ______                 \n\
                                                         (_)/ ____/__  __ / ____/____   _____ ___  \n\
                                 ______ ______ ______   / // /_   / / / // /    / __ \\ / ___// _ \\ \n\
                                /_____//_____//_____/  / // __/  / /_/ // /___ / /_/ // /   /  __/ \n\
                                                      /_//_/     \\__,_/ \\____/ \\____//_/    \\___/  \n\
                                                                                                   \n";

/* We come here after U-Boot is initialised and ready to process commands */
void main_loop(void)
{
	const char *s;

	bootstage_mark_name(BOOTSTAGE_ID_MAIN_LOOP, "main_loop");

	if (IS_ENABLED(CONFIG_VERSION_VARIABLE))
		env_set("ver", version_string);  /* set version variable */

	cli_init();
	run_preboot_environment_command();

	if (IS_ENABLED(CONFIG_UPDATE_TFTP))
		update_tftp(0UL, NULL, NULL);

	s = bootdelay_process();
	if (cli_process_fdt(&s))
		cli_secure_boot_cmd(s);

	autoboot_command(s);

    printf("%s", logo_1); printf("%s", logo_2);
	cli_loop();
	panic("No CLI available");
}
