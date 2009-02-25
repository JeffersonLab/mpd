/* Event Builder Data Decoder
 */
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>

#define BLOCK_HEADER    0x0	// {3'h0, MODULE_ID, EVENT_PER_BLOCK, BLOCK_CNT[7:0]}
#define BLOCK_TRAILER   0x1	// {3'h1, 1'b0, BlockWordCounter}
#define EVENT_HEADER    0x2	// {3'h2, 1'b0, EventCounterFifo_Data}
#define TRIGGER_TIME1   0x3	// {3'h3, 1'b0, TimeCounterFifo_Data[39:20]}
#define TRIGGER_TIME2   0x3	// {3'h3, 1'b1, TimeCounterFifo_Data[19:0]}
#define APV_CH_DATA     0x4	// {3'h4, ChannelData[20:0]}
#define EVENT_TRAILER   0x5	// {3'h5, 1'b0, LoopDataCounter[11:0], TRIGGER_TIME_FIFO}
#define DATA_NOT_VALID  0x6	// {3'h6, 21'b0}
#define FILLER_WORD     0x7	// {3'h7, 21'b0}


main(int argc, char **argv)
{
	FILE *f;
	int pos, ModuleId, EventPerBlock, BlockCounter, BlockWordCounter, EventCounter;
	int BlockCounter_prev, EventCounter_prev;
	int EventTime0, EventTime1, ChannelData, LoopDataCounter, TriggerTime;
	int Data_Hdr_MeanMSB, Data_Hdr_ApvHdr, Data_Hdr_AdcCh, ApvData_ch, ApvData_val, Data_Mean;
	int ApvFrame_Trl_ModuleId, ApvFrame_Trl_SampleCnt, ApvFrame_Trl_FrameCnt;
	int Data_Trl_Mean, Data_Trl_Wc;
	int blk_hdr, blk_trl, evt_hdr, evt_trl, evt_time0, evt_time1, not_valid, filler, apv_data;
	int blk_hdr_seq, blk_trl_seq, evt_hdr_seq, evt_trl_seq;
	int hdr_trl_badseq, ev_cnt_badseq, blk_cnt_badseq;
	int print_apv_data, all_apv_channels, hex_data, c, stat_only;
	uint32_t data, channel_enable_mask, n_samples, n_apv;
	char *filemode;
	int apv_frame_count, apv_chan_count, blk_word_count, loop_data_count, frame_count_err;

	if( argc < 2 )
	{
		printf("Usage: %s [-a] [-c] [-x] fname\n", argv[0]);
		exit(1);
	}

	opterr = 0;
	hex_data = print_apv_data = all_apv_channels = stat_only = 0;
	filemode = "rb";
	while ((c = getopt (argc, argv, "acxhs")) != -1)
		switch (c)
		{
			case 'a':
				print_apv_data = 1;
				break;
			case 'c':
				print_apv_data = 1;
				all_apv_channels = 1;
				break;
			case 'x':
				hex_data = 1;
				filemode = "r";
				break;
			case 'h':
				printf("Usage: %s [-a] [-c] [-x] [-h] [-s] fname\n", argv[0]);
				printf("\t-a: print APV data summary\n");
				printf("\t-c: print all APV data\n");
				printf("\t-x: use ASCII hex file\n");
				printf("\t-h: print this help\n");
				printf("\t-s: print only statistic summary\n");
				exit(1);
				break;
			case 's':
				stat_only = 1;
				print_apv_data = 0;
				all_apv_channels = 0;
				break;
			default:
				break;
		}

	if( (f = fopen(argv[optind],filemode)) == NULL )
	{
		printf("%s: can't open '%s'\n", argv[0], argv[argc-1]);
		exit(1);
	}

	pos = 0;
	blk_hdr =  blk_trl =  evt_hdr =  evt_trl =  evt_time0 =  evt_time1 =  not_valid =  filler =  apv_data = 0;
	blk_hdr_seq =  blk_trl_seq =  evt_hdr_seq =  evt_trl_seq =  0;
	hdr_trl_badseq = ev_cnt_badseq = blk_cnt_badseq = 0;
	EventCounter_prev = BlockCounter_prev = 0;
	frame_count_err = 0;

	if( hex_data )
	{
		fscanf(f, "%x", &channel_enable_mask);
		fscanf(f, "%x", &n_samples);
		fscanf(f, "%x", &data);
	}
	else
	{
		fread(&channel_enable_mask, sizeof(uint32_t), 1, f);
		fread(&n_samples, sizeof(uint32_t), 1, f);
		fread(&data, sizeof(uint32_t), 1, f);
	}
	channel_enable_mask &= 0xFFFF;
	n_samples &= 0xFFFF;
	n_apv = 0;
	for(int i=0; i<16; i++)
		if( channel_enable_mask & (1<<i) )
			n_apv++;
//	printf("Channel Enable Mask = 0x%04X, N. Samples = %d, N. APV = %d\n\n", channel_enable_mask, n_samples, n_apv);

	while( !feof(f) )
	{
//		if( pos < 10 )
//			printf("***** Data[%d] = 0x%08x\n", pos, data);
		data &= 0x0FFFFFFF;
		switch( (data & 0x00E00000) >> 21 )
		{
			case BLOCK_HEADER:	// 0x0
				ModuleId      = (data & 0x001F0000) >> 16;
				EventPerBlock = (data & 0x0000FF00) >> 8;
				BlockCounter  = (data & 0x000000FF);
				if( stat_only == 0 )
					printf("@%06d BlockHeader: ModuleID = %d, EventPerBlock = %d, BlockCounter = %d\n",
						pos, ModuleId, EventPerBlock, BlockCounter);
				if( BlockCounter != ((BlockCounter_prev + 1) % 256) && pos > 0 )
				{
					blk_cnt_badseq++;
					if( stat_only == 0 )
						printf("\tSEQ ERROR@BlockCounter = %d\n\n", BlockCounter);
				}
				blk_hdr++;
				blk_hdr_seq++;
				BlockCounter_prev = BlockCounter;
				blk_word_count = 0;
				break;
			case BLOCK_TRAILER:	// 0x1
				BlockWordCounter  = (data & 0x000FFFFF);
				if( stat_only == 0 )
					printf("@%06d BlockTrailer: BlockWordCounter = %d (counted = %d)\n\n", pos, BlockWordCounter, blk_word_count);
				blk_trl++;
				blk_trl_seq++;
				if( blk_hdr_seq != 1 || blk_trl_seq != 1 || evt_hdr_seq != 1 || evt_trl_seq != 1 )
				{
					hdr_trl_badseq++;
					if( stat_only == 0 )
						printf("\tSEQ ERROR@BlockTrailer = %d: BlkHdr_seq = %d BlkTrl_seq = %d EvtHdr_seq = %d EvtTrl_seq = %d\n\n",
							blk_trl, blk_hdr_seq, blk_trl_seq, evt_hdr_seq, evt_trl_seq);
				
				}
				blk_hdr_seq =  blk_trl_seq =  evt_hdr_seq =  evt_trl_seq =  0;
				break;
			case EVENT_HEADER:	// 0x2
				EventCounter  = (data & 0x000FFFFF);
				if( stat_only == 0 )
					printf("@%06d EventHeader: EventCounter = %d\n", pos, EventCounter);
				if( EventCounter != (EventCounter_prev + 1) )
				{
					ev_cnt_badseq++;
					if( stat_only == 0 )
						printf("\tSEQ ERROR@EventHeader = %d\n\n", EventCounter);
				}
				evt_hdr++;
				evt_hdr_seq++;
				EventCounter_prev = EventCounter;
				blk_word_count++;
				loop_data_count++;
				apv_frame_count = 0;
				break;
			case TRIGGER_TIME1:	// 0x3
				if( data & 0x00100000 ) {
					EventTime1 = (data & 0x000FFFFF);
					if( stat_only == 0 )
						printf("@%06d EventTime1: Time1 = %d\n", pos, EventTime1);
					evt_time1++;
					loop_data_count = 0;
					}
				else {
					EventTime0 = (data & 0x000FFFFF);
					if( stat_only == 0 )
						printf("@%06d EventTime0: Time0 = %d\n", pos, EventTime0);
					evt_time0++;
					}
				blk_word_count++;
				break;
			case APV_CH_DATA:	// 0x4
				loop_data_count++;
				blk_word_count++;
				switch( (data & 0x000180000) >> 19 )
				{
					case 0:
//fifo_data_in <= {1'b0, 1'b0, 1'b0, MEAN[11], DATA_IN[12:0], CH_ID[3:0]};	// APV processor header
						Data_Hdr_MeanMSB = (data & 0x00020000) >> 6;
						Data_Hdr_ApvHdr  = (data & 0x0001FFF0) >> 4;
						Data_Hdr_AdcCh   = (data & 0x0000000F);
						if( print_apv_data )
							printf("@%06d ApvProcessor_Header: MeanMSB = %d, ApvHdr = 0x%x, AdcCh = %d\n",
								pos, Data_Hdr_MeanMSB, Data_Hdr_ApvHdr, Data_Hdr_AdcCh);
//						printf("\t\tApvProcessor_Header RawData = 0X%06X\n", data);
						apv_chan_count = 0;
						break;
					case 1:
//fifo_data_in <= {1'b0, 1'b1, THRESHOLD_ADDRESS[6:0], data_minus_baseline[11:0]};	// APV data
						ApvData_ch  = (data & 0x0007F000) >> 12;
						ApvData_val = (data & 0x00000FFF);
						if( all_apv_channels )
							printf("@%06d ApvDataRaw: Apv Ch = %d, Data = 0x%x\n",
								pos, ApvData_ch, ApvData_val);
						else
							if( print_apv_data && (ApvData_ch == 0 || ApvData_ch == 127) )
								printf("@%06d ApvDataRaw: Apv Ch = %d, Data = 0x%x\n",
									pos, ApvData_ch, ApvData_val);
//						printf("\t\tApv RawData = 0X%06X\n", data);
						apv_chan_count++;
						break;
					case 2:
//fifo_data_in <= {1'b1, 1'b0, 2'b0, MODULE_ID[4:0], DATA_IN[11:0]};	// APV frame decoder trailer
// DATA_IN[11:0] = {ApvSampleCounterMinusOne[3:0], frame_counter[7:0]};
						ApvFrame_Trl_ModuleId  = (data & 0x0001F000) >> 12;	// MPD ID
						ApvFrame_Trl_SampleCnt = (data & 0x00000F00) >> 8;// 0..SAMPLE_PER_EVENT-1
						ApvFrame_Trl_FrameCnt  = (data & 0x000000FF);	// APV header count % 256
						if( print_apv_data )
							printf("@%06d ApvDecoder_Trailer: ModuleID = %d, Sample Count = %d, Frame Count = %d, Channel count in frame = %d\n",
								pos, ApvFrame_Trl_ModuleId, ApvFrame_Trl_SampleCnt, ApvFrame_Trl_FrameCnt, apv_chan_count);
//						printf("\t\tApvDecoder_Trailer RawData = 0X%06X\n", data);
						break;
					case 3:
//fifo_data_in <= {1'b1, 1'b1, MEAN[10:0], word_count[7:0]};	// baseline subtractor trailer
						Data_Trl_Mean = (data & 0x0007FF00) >> 8;
						Data_Trl_Wc   = (data & 0x000000FF);
						Data_Mean     = Data_Trl_Mean + Data_Hdr_MeanMSB;
						if( print_apv_data )
							printf("@%06d ApvProcessor_Trailer: MeanLSB = %d, WordCount = %d, Mean = %d\n",
								pos, Data_Trl_Mean, Data_Trl_Wc, Data_Mean);
//						printf("\t\tApvProcessor_Trailer RawData = 0X%06X\n", data);
						apv_frame_count++;
						break;
				}
				apv_data++;
				break;
			case EVENT_TRAILER:	// 0x5
				LoopDataCounter  = (data & 0x000FFF00) >> 8;
				TriggerTime      = (data & 0x000000FF);
				if( stat_only == 0 )
					printf("@%06d EventTrailer: LoopDataCounter = %d (counted = %d), TriggerTdcTime = %d (APV frame counted = %d)\n",
						pos, LoopDataCounter, loop_data_count%0xFFF, TriggerTime, apv_frame_count);
				if( apv_frame_count != (n_samples*n_apv) )
				{
					frame_count_err++;
					if( stat_only == 0 )
						printf("\tAPV FRAME COUNT ERROR: Counted = %d, expected = %d\n\n", apv_frame_count, n_samples*n_apv);
				}
				evt_trl++;
				evt_trl_seq++;
				blk_word_count++;
				break;
			case DATA_NOT_VALID:	// 0x6
				not_valid++;
				if( stat_only == 0 )
					printf("@%06d DataNotValid\n", pos);
				blk_word_count++;
				break;
			case FILLER_WORD:	// 0x7
				filler++;
				if( stat_only == 0 )
					printf("@%06d FillerWord\n", pos);
				blk_word_count++;
				break;
		}
		pos++;
		if( hex_data )
			fscanf(f, "%x", &data);
		else
			fread(&data, sizeof(uint32_t), 1, f);
	}
	fclose(f);
	printf("\n\nTotal word read = %d\n\n", pos);
	printf("BlockHeader_count = %d, BlockTrailer_count = %d\n", blk_hdr, blk_trl);
	printf("EventHeader_count = %d, EventTrailer_count = %d\n", evt_hdr, evt_trl);
	printf("Time0_count = %d, Time1_count = %d\n", evt_time0, evt_time1);
	printf("NotValid_count = %d, Filler_count = %d\n", not_valid, filler);
	printf("ApvData_count = %d\n", apv_data);
	printf("Header-Trailer sequence error count = %d\n", hdr_trl_badseq);
	printf("Event Counter sequence error count = %d\n", ev_cnt_badseq);
	printf("Block Counter sequence error count = %d\n", blk_cnt_badseq);
	printf("Frame Counter error count = %d\n", frame_count_err);
}

