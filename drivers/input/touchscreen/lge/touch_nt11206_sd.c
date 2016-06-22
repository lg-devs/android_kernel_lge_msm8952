/* touch_nt11206_sd.c
 *
 * Copyright (C) 2015 LGE.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include "touch_hwif.h"
#include "touch_core.h"
#include "touch_nt11206.h"
#include <mach/board_lge.h>


#define PSConfig_Tolerance_Negative 200
#define PSConfig_Tolerance_Postive 200
#define PSConfig_TPSelectNegativePercentage 200
#define PSConfig_TPSelectPositivePercentage 200
#define PSConfig_DiffLimitG_Negative -200
#define PSConfig_DiffLimitG_Postive 200
long int boundary[30*20] = {
	1956,1949,1937,1923,1927,1926,1923,1921,1928,1931,1883,1896,1894,1892,1913,1928,1942,1956,1983,2009,
	831,840,846,842,846,849,847,850,856,858,809,818,820,822,831,848,850,859,872,876,
	1206,1223,1231,1231,1232,1235,1236,1235,1241,1238,1228,1225,1226,1229,1225,1241,1241,1242,1245,1237,
	1274,1270,1274,1271,1269,1269,1271,1269,1272,1271,1264,1261,1262,1265,1259,1281,1280,1283,1288,1292,
	524, 527,533,533,535,537,536,538,540,541,514,518,519,520,524,538,538,541,548,550,
	392,399,400,401,406,406,406,407,411,410,376,381,381,382,393,400,403,407,413,417,
	380,384,384,383,385,386,386,386,388,385,372,375,375,374,387,389,390,391,391,393,
	548,553,555,553,554,556,556,556,558,557,541,545,546,545,551,562,562,564,567,568,
	1223,1224,1228,1225,1223,1224,1225,1224,1227,1226,1216,1221,1221,1223,1217,1238,1235,1237,1240,1241,
	1215,1214,1214,1212,1209,1210,1212,1210,1213,1214,1204,1204,1204,1207,1204,1224,1222,1224,1226,1228,
	540,546,545,544,545,546,546,547,548,548,530,534,535,535,547,554,554,556,556,558,
	367,369,369,368,370,370,371,370,371,370,355,358,359,360,373,374,376,377,374,378,
	357,356,356,355,356,356,357,356,357,355,345,348,348,352,362,362,363,363,360,362,
	516,520,520,519,519,521,521,522,523,523,508,512,513,513,525,530,529,531,527,530,
	1112,1117,1117,1114,1113,1114,1115,1115,1118,1118,1106,1110,1110,1112,1114,1129,1125,1127,1127,1129,
	1063,1060,1061,1060,1059,1060,1061,1061,1064,1065,1056,1054,1055,1058,1060,1074,1071,1073,1071,1078,
	488,489,490,489,490,492,491,492,493,493,478,481,482,486,494,499,499,500,497,502,
	342,342,342,342,343,343,344,344,344,342,332,334,334,343,348,348,349,348,346,349,
	302,304,305,306,309,310,310,310,311,310,298,300,301,310,312,312,314,313,314,315,
	440,446,446,448,450,453,452,454,455,455,439,442,443,451,453,458,458,459,460,460,
	930,937,937,937,937,939,941,941,945,947,930,934,934,939,939,952,950,952,954,954,
	902,898,899,898,898,901,904,905,909,911,903,901,901,909,903,915,913,915,915,920,
	434,434,435,435,436,440,442,444,445,446,433,434,436,442,444,447,447,448,449,451,
	308,307,307,307,310,312,315,315,316,315,305,306,309,314,317,317,318,318,319,320,
	295,295,294,294,295,296,298,300,302,302,293,294,301,302,303,303,304,304,305,305,
	408,410,410,409,410,412,413,416,420,421,408,410,413,417,418,421,421,422,423,425,
	781,782,782,781,781,783,785,786,792,797,782,785,784,791,787,798,795,797,798,802,
	765,769,767,769,767,776,768,774,775,793,793,784,785,791,789,800,794,798,797,800,
	413,411,416,410,416,416,417,414,421,424,412,410,417,417,420,422,420,422,422,423,
	312,318,314,318,315,322,319,323,321,330,316,314,321,319,323,321,322,322,321,324,
};

#define MaxStatisticsBuf 100
static long int golden_Ratio[30*20] = {0, };
static int RawDataTest_Sub(__s32 *rawdata, u8 *RecordResult,u8 x_num, u8 y_num);
static int Test_CaluateGRatioAndNormal(__s32* rawdata, u8 x_num, u8 y_num);
static int nvt_read_baseline(struct device *dev, __s32* xdata)
{
	u8 x_num = 0;
	u8 y_num = 0;

	nvt_change_mode(dev, TEST_MODE_1);

	if(nvt_check_fw_status(dev) != 0)
		return -EAGAIN;

	nvt_get_fw_info(dev);

	nvt_read_mdata(dev, BASELINE_ADDR);

	nvt_get_mdata(xdata, &x_num, &y_num);

	nvt_change_mode(dev, MODE_CHANGE_NORMAL_MODE);

	return 0;
}
int nt11206_selftest(struct device *dev, char* buf, u8 mode)
{
	struct nt11206_data *d = to_nt11206_data(dev);
	__s32* xdata = NULL;
	u8* record_result = NULL;
	u8 x_num=0;
	u8 y_num=0;
	int sd_ret = 0;
	int ret = 0;
	u8 i = 0;
	u8 j = 0;

	x_num = d->fw.x_axis_num;
	y_num = d->fw.y_axis_num;

	if(	d->resume_state == 0 && !mode) {
		TOUCH_E("LCD OFF, mode:%d\n", mode);
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "LCD OFF\n");
		return ret;
	}

	xdata = (__s32*)kmalloc(sizeof(__s32) * 20 * 40, GFP_KERNEL);
	if(xdata == NULL) {
		sd_ret = -1;
		TOUCH_E("rawdata Alloc Failed\n");
		return sd_ret;
	}
	record_result = (u8*)kmalloc(d->fw.x_axis_num * d->fw.y_axis_num, GFP_KERNEL);
	if(record_result == NULL) {
		sd_ret = -1;
		TOUCH_E("record_result Alloc Failed\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "record_result Alloc Failed\n");
		if(xdata) {
			kfree(xdata);
		}
		return ret;
	}
	memset(xdata, 0, sizeof(__s32) * 20 * 30);
	memset(record_result, 0, d->fw.x_axis_num * d->fw.y_axis_num);
	TOUCH_I("x_axis_num:%d, y_axis_num:%d\n", d->fw.x_axis_num, d->fw.y_axis_num);
	if((x_num != 20) || (y_num != 30)) {
		TOUCH_E("FW Info is broken\n");
		x_num = 20;
		y_num = 30;
	}

	if(nvt_read_baseline(dev, xdata) != 0) {
		sd_ret = 1;
	}
	else {
		//---Self Test Check ---
    	sd_ret = RawDataTest_Sub(xdata, record_result, x_num, y_num);	// 0:PASS, -1:FAIL
	}

	ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Channel Status : Pass\n");
	switch(sd_ret) {
		case 0:
			TOUCH_I("Self Test PASS!\n");
			if(!mode) {
				ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Raw Data : Pass\n");
			}
			else {
				ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "LPWG RawData : Pass\n");
			}
			TOUCH_I("RecordResult:\n");
		    for(i=0; i<y_num; i++)
		    {
		        for(j=0; j<x_num; j++)
		        {
		            printk("0x%02X, ", record_result[i*x_num+j]);
					ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "0x%02X, ", record_result[i*x_num+j]);
		        }
				printk("\n");
				ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "\n");
		    }
			TOUCH_I("ReadData:\n");
		    for(i=0; i<y_num; i++)
		    {
		        for(j=0; j<x_num; j++)
		        {
		            printk("%5d, ", xdata[i*x_num+j]);
		        }
				printk("\n");
		    }
			break;
			
		case 1:
			TOUCH_E("Self Test ERROR! Read Data FAIL!\n");
			if(!mode) {
				ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Raw Data : Fail\n");
			}
			else {
				ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "LPWG RawData : Fail\n");
			}
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Self Test ERROR! Read Data FAIL!\n");
			break;

		case -1:
			TOUCH_E("Self Test FAIL!\n");
			if(!mode) {
				ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Raw Data : Fail\n");
			}
			else {
				ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "LPWG RawData : Fail\n");
			}
			TOUCH_I("RecordResult:\n");
		    for(i=0; i<y_num; i++)
		    {
		        for(j=0; j<x_num; j++)
		        {
		            printk("0x%02X, ", record_result[i*x_num+j]);
					ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "0x%02X, ", record_result[i*x_num+j]);
		        }
				printk("\n");
				ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "\n");
		    }
			TOUCH_I("ReadData:\n");
		    for(i=0; i<y_num; i++)
		    {
		        for(j=0; j<x_num; j++)
		        {
		            printk("%5d ", xdata[i*x_num+j]);
		        }
				printk("\n");
		    }
			break;
	}

	if(xdata)
		kfree(xdata);

	if(record_result)
		kfree(record_result);

	return ret;
}

static int Test_CaluateGRatioAndNormal(__s32* rawdata, u8 x_num, u8 y_num)
{
	int i, j, k;
	long int tmpValue;
	long int MaxSum=0;
	int MaxNum=0, MaxIndex=0;
	int Max = -99999;
	int Min =  99999;
	int offset;
	int Data;
	int StatisticsStep=0;
	long int StatisticsNum[MaxStatisticsBuf];
	long int StatisticsSum[MaxStatisticsBuf];
    
	//--------------------------------------------------
	//1. (Testing_CM - Golden_CM ) / Testing_CM
	//--------------------------------------------------
	for(j=0; j<y_num; j++)
	{
		for(i=0; i<x_num; i++)
		{
			Data = rawdata[j*x_num + i];
			if(Data == 0)
				Data = 1;

			golden_Ratio[j*x_num + i] = Data - boundary[j*x_num + i];
			golden_Ratio[j*x_num + i] = ((golden_Ratio[j*x_num + i]*1000) / Data);	// *1000 before division
		}
	}
    
	//--------------------------------------------------------
	// 2. Mutual_GoldenRatio*1000
	//--------------------------------------------------------
	for(j=0; j<y_num; j++)
	{
		for(i=0; i<x_num; i++)
		{
			golden_Ratio[j*x_num + i] *= 1000;
		}
	}
    
	//--------------------------------------------------------
	// 3. Calculate StatisticsStep
	//--------------------------------------------------------
	for(j=0; j<y_num; j++)
	{
		for(i=0; i<x_num; i++)
		{
			if (Max < golden_Ratio[j*x_num + i])
				Max = (int)golden_Ratio[j*x_num + i];
			if (Min > golden_Ratio[j*x_num + i])
				Min = (int)golden_Ratio[j*x_num + i];
		}
	}

	offset = 0;
	if(Min < 0) // add offset to get erery element Positive
	{
		offset = 0 - Min;	
		for(j=0; j<y_num; j++)
		{
			for(i=0; i<x_num; i++)
			{
				golden_Ratio[j*x_num + i] += offset;
			}
		}
		Max += offset;
	}
	StatisticsStep = Max / MaxStatisticsBuf;
	StatisticsStep += 1;
	if(StatisticsStep < 0)
	{
		TOUCH_E("FAIL! (StatisticsStep < 0)\n");
		return 1;
	}
	
	//--------------------------------------------------------
	// 4. Start Statistics and Average
	//--------------------------------------------------------
	memset(StatisticsSum, 0, sizeof(long int)*MaxStatisticsBuf);
	memset(StatisticsNum, 0, sizeof(int)* MaxStatisticsBuf);
	for(i=0; i<MaxStatisticsBuf; i++)
	{
		StatisticsSum[i] = 0;
		StatisticsNum[i] = 0;
	}
	for(j=0; j<y_num; j++)
	{
		for(i=0; i<x_num; i++)
		{
			tmpValue = golden_Ratio[j*x_num + i];
			if(!StatisticsStep) {
				TOUCH_E("DIV 0\n");
				return 0;
			}
			tmpValue /= StatisticsStep;
			StatisticsNum[tmpValue] += 2;
			StatisticsSum[tmpValue] += (2*golden_Ratio[j*x_num + i]);

			if((tmpValue + 1) < MaxStatisticsBuf)
			{
				StatisticsNum[tmpValue+1] += 1;
				StatisticsSum[tmpValue + 1] += golden_Ratio[j*x_num + i];
			}
			//if((tmpValue - 1) >= 1)
			//{
				if ((tmpValue - 1) < MaxStatisticsBuf)
				{
					StatisticsNum[tmpValue - 1] += 1;
					StatisticsSum[tmpValue - 1] += golden_Ratio[j*x_num + i];
				}
			//}
			//else
			//{
			//	mPSConfig.myLayerPrint(StrMessageLevel.WarningMsg, "Statistics Index = " + (tmpValue - 1).ToString()); 
			//}
		}
	}
	//Find out Max Statistics
	MaxNum =0;
	for(k=0; k<MaxStatisticsBuf; k++)
	{
		if(MaxNum < StatisticsNum[k])
		{
			MaxSum = StatisticsSum[k];
			MaxNum = StatisticsNum[k];	
			MaxIndex = k;
		}			
	}
	//Caluate Statistics Average
	if(MaxSum > 0)
	{
		if(!StatisticsNum[MaxIndex]) {
			TOUCH_E("DIV 0\n");
			return 0;
		}
		tmpValue = (StatisticsSum[MaxIndex] / StatisticsNum[MaxIndex])*2;
		if((MaxIndex+1) < (MaxStatisticsBuf)) {
			if(!StatisticsNum[MaxIndex+1]) {
				TOUCH_E("DIV 0\n");
				return 0;
			}
			tmpValue += (StatisticsSum[MaxIndex+1] / StatisticsNum[MaxIndex+1]);
		}
		if((MaxIndex-1) >= 0 ) {
			if(!StatisticsNum[MaxIndex-1]) {
				TOUCH_E("DIV 0\n");
				return 0;
			}
			tmpValue += (StatisticsSum[MaxIndex-1] / StatisticsNum[MaxIndex-1]);
		}

		if((MaxIndex+1) < (MaxStatisticsBuf) &&( (MaxIndex-1) >=0)) 
			tmpValue /=4;
		else
			tmpValue /=3;
	}
	else // Too Separately
	{
		StatisticsSum[0] = 0;
		StatisticsNum[0] = 0;
		for(j=0; j<y_num; j++)
		{
			for(i=0; i<x_num; i++)
			{
				StatisticsSum[0] += (long int)golden_Ratio[j*x_num + i];
				StatisticsNum[0]++;
			}
		}
		if(!StatisticsNum[0]) {
			TOUCH_E("DIV 0\n");
			return 0;
		}
		tmpValue = StatisticsSum[0] / StatisticsNum[0];
	}
	//----------------------------------------------------------
	//----------------------------------------------------------
	//----------------------------------------------------------
	tmpValue -= offset;
	for(j= 0; j<y_num; j++)
	{
		for(i=0; i<x_num; i++)
		{
			golden_Ratio[j*x_num + i] -= offset;

			golden_Ratio[j*x_num + i] = golden_Ratio[j*x_num + i] - tmpValue;
			golden_Ratio[j*x_num + i] = golden_Ratio[j*x_num + i] / 1000;
		}
	}

	return 0;
}


static int RawDataTest_Sub(__s32 *rawdata, u8 *RecordResult,u8 x_num, u8 y_num)
{
	int i, j;
	int kk0=0, kk1=0;
	int kkTPSelectLB = 0;
	int kkTPSelectUB = 0;
	int kkTPSelect = 0;

	//--------------------------------------------------------
	// Init RecordResult array
	//--------------------------------------------------------
	for(j=0; j<y_num; j++)
	{
		for (i=0; i<x_num; i++)
		{
			RecordResult[j*x_num + i] = 0;
		}
	}

	//--------------------------------------------------------
	// 5a. Check abs low boundary
	//--------------------------------------------------------
	for(j=0; j<y_num; j++)
	{
		for(i=0; i<x_num; i++)
		{
			kk0 = boundary[j*x_num + i];
			if (kk0 > 0)
			{
				kk0 = (int)((boundary[j*x_num + i]*(1000-PSConfig_Tolerance_Negative)) / 1000);	// *1000 before division
			}
			else
			{
				kk0 = (int)((boundary[j*x_num + i]*(1000+PSConfig_Tolerance_Negative)) / 1000);	// *1000 before division
			}

			if (rawdata[j*x_num + i] < kk0)
			{
				RecordResult[j*x_num + i] |= 0x02;
			}
			else
			{
				kk1 = boundary[j*x_num + i];
				if (kk1 > 0)
				{
					kk1 = (int)((boundary[j*x_num + i]*(1000+PSConfig_Tolerance_Postive)) / 1000);	// *1000 before division
				}
				else
				{
					kk1 = (int)((boundary[j*x_num + i]*(1000-PSConfig_Tolerance_Postive)) / 1000);	// *1000 before division
				}

				if (rawdata[j*x_num + i] > kk1)
				{
					RecordResult[j*x_num + i] |= 0x01;
				}
			}
		}
	}

	//--------------------------------------------------------
	// 5b. Choose the selection TP to verify the rationality of "Golden Sample Boundary"
	//--------------------------------------------------------
	for(j=0; j<y_num; j++)
	{
		for(i=0; i<x_num; i++)
		{
			kkTPSelectLB = (int)((boundary[j*x_num + i]*(1000-PSConfig_TPSelectNegativePercentage)) / 1000);	// *1000 before division
			kkTPSelectUB = (int)((boundary[j*x_num + i]*(1000+PSConfig_TPSelectPositivePercentage)) / 1000);	// *1000 before division

			if (rawdata[j*x_num + i] < kkTPSelectLB)
			{
				RecordResult[j*x_num + i] |= 0x04;
			}
			else if (rawdata[j*x_num + i] > kkTPSelectUB)
			{
				RecordResult[j*x_num + i] |= 0x04;
			}
		}
	}

	//--------------------------------------------------
	//6. (Testing_CM - Golden_CM ) / Testing_CM
	//--------------------------------------------------
	Test_CaluateGRatioAndNormal(rawdata, x_num, y_num);

	//--------------------------------------------------------
	// 7 . Check Golden Ratio Test
	//--------------------------------------------------------
	for(j=0; j<y_num; j++)
	{
		for(i=0; i<x_num; i++)
		{
			if(golden_Ratio[j*x_num + i] < PSConfig_DiffLimitG_Negative)
			{
				RecordResult[j*x_num + i] |= 0x08;
			}
			if(golden_Ratio[j*x_num + i] > PSConfig_DiffLimitG_Postive)
			{
				RecordResult[j*x_num + i] |= 0x04;
			}
		}
	}

	//--------------------------------------------------------
	// 8 . Record Test Result & Select TP Panel
	//--------------------------------------------------------
	kk0 = 0;
	kkTPSelect = 0;
	for(j=0; j<y_num; j++)
	{
		for(i=0; i<x_num; i++)
		{
			if((RecordResult[j*x_num + i] & 0x01) > 0)
				kk0++;
			if((RecordResult[j*x_num + i] & 0x02) > 0)
				kk0++;
			if((RecordResult[j*x_num + i] & 0x04) > 0)
				kk0++;
			if((RecordResult[j*x_num + i] & 0x08) > 0)
				kk0++;
		}
	}

	if(kk0 >= 1)
	{
		return -1;	// FAIL
	}
	else
	{
		return 0;	// PASS
	}          
}
