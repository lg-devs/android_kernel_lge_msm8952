#include <linux/kernel.h>
#include <soc/qcom/lge/lge_display_panel_check.h>

int display_panel_type;

void lge_set_panel(int panel_type)
{
	pr_info("panel_type is %d\n", panel_type);

	display_panel_type = panel_type;
}

int lge_get_panel(void)
{
	return display_panel_type;
}
