#include "elog_interface.h"




void elog_func_init(void)
{
	elog_init();
	
	elog_set_text_color_enabled(true);
	
	elog_set_fmt(ELOG_LVL_ASSERT,ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_FUNC | ELOG_FMT_LINE | ELOG_FMT_TIME);
	
	elog_set_fmt(ELOG_LVL_ERROR,ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_FUNC | ELOG_FMT_LINE | ELOG_FMT_TIME);

	elog_set_fmt(ELOG_LVL_WARN,ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_FUNC);

	elog_set_fmt(ELOG_LVL_INFO,ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
	
	elog_set_fmt(ELOG_LVL_DEBUG,ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
	
	elog_set_fmt(ELOG_LVL_VERBOSE,ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
	
    if(ELOG_ENABLE == 1)
    {
        elog_start();
    }
    else
    {

    }


	

	
}

/**
 * EasyLogger demo
 */
void test_elog2(void) 
{
	static uint32_t cnt = 0;
	

    /* test log output for all level */
    elog_a("ASSERT","Hello EasyLogger!%d",cnt);
    elog_e("ERROR","Hello EasyLogger!%d",cnt);
    elog_w("WARN","Hello EasyLogger!%d",cnt);
    elog_i("INFO","Hello EasyLogger!%d",cnt);
    elog_d("DEBUG","Hello EasyLogger!%d",cnt);
    elog_v("VERBOSE","Hello EasyLogger!%d",cnt);
//    elog_raw("Hello EasyLogger!");

	cnt++;
}

void test_elog(void) 
{
	static uint32_t cnt = 0;
	

    /* test log output for all level */
    ELOG_A("ASSERT","Hello EasyLogger!%d",cnt);
    ELOG_E("ERROR","Hello EasyLogger!%d",cnt);
    ELOG_W("WARN","Hello EasyLogger!%d",cnt);
    ELOG_I("INFO","Hello EasyLogger!%d",cnt);
    ELOG_D("DEBUG","Hello EasyLogger!%d",cnt);
    ELOG_V("VERBOSE","Hello EasyLogger!%d",cnt);
//    elog_raw("Hello EasyLogger!");

	cnt++;
}
