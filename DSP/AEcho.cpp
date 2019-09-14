#include <Pismo.h>
#include <ctype.h>
#include <cstdlib>
using namespace II;

#include "initialization.h"
#include "scheduler.h"
#include "constants.h"
#include "AEcho.h"
//#include "C:\\CCStudio_v3.3\\C6000\\cgtools\\include\\math.h"


void disable_IRQ_func();
//---------------------------------------------------------------------------
//  IsrHandler --  interrupt handler
//---------------------------------------------------------------------------
class IsrHandler
{
public:
	IsrHandler()
		: Binder(*this, &IsrHandler::MyHandler, &ISR2KHz_counter), ISR2KHz_counter(0){ }
	ClassMemberHandler<IsrHandler, unsigned int *> Binder; 

	void MyHandler(unsigned int * tally)
	{
		*tally += 1;
		Read_Interrupts();
	}
	
	unsigned int 	ISR2KHz_counter;
};

// Instantiate a concrete instance of above class..
IsrHandler  Isr;
Irq *p_myIRQ;


class IsrHandler11ms
{
public:
	IsrHandler11ms()
		: Binder(*this, &IsrHandler11ms::MyHandler, &ISR11ms_counter), ISR11ms_counter(0)  { }
	ClassMemberHandler<IsrHandler11ms, unsigned int *> Binder; 


        void MyHandler(unsigned int * tally)
        {  
                 // do my processing for Int5, including enabling nested interrupts
        }        

	unsigned int 	ISR11ms_counter;
	      
};
IsrHandler11ms Isr11ms;
Irq *p_IRQ11ms;



///////////////////////////////////////////////////////////////////////////
typedef struct
{
    unsigned int    Ier;
    unsigned int    Irp;
    unsigned int    Csr;
} NestISRContext;

//------------------------------------------------------------------------  
//  enable_nested_interrupts() 
//      Saves the additional context registers required to next ISRs
//      and enables interrupts 
//------------------------------------------------------------------------ 
 
__INLINE void  enable_nested_interrupts(NestISRContext * context)
{
    //
    //  Save registers in local context structure
    context->Ier = IER;
    context->Irp = IRP;
    context->Csr = CSR;
    IRQ_enable(IRQ_EVT_EXTINT5);//enable_interrupts();
}

//------------------------------------------------------------------------  
//  disable_nested_interrupts() 
//      Disables interrupts and restores the additional context 
//      registers required to next ISRs and enables interrupts 
//------------------------------------------------------------------------ 
 
__INLINE void  disable_nested_interrupts(NestISRContext * context)
{
    IRQ_disable(IRQ_EVT_EXTINT5); // disable_interrupts();
    //
    //  Restore registers from local context structure
    IER = context->Ier;
    IRP = context->Irp;
    CSR = context->Csr;
}


void interrupt BusIsr()
{
    NestISRContext local_context;    

    IRQ_disable(IRQ_EVT_EXTINT5);// DisableInterrupt(BUS_INTERRUPT); // disable this interrupt so it cannot be interrupted by itself
    enable_nested_interrupts(&local_context);

    /*
        do some processing
    */
    
    disable_nested_interrupts(&local_context);        
    IRQ_enable(IRQ_EVT_EXTINT5);//EnableInterrupt(BUS_INTERRUPT);
}
// these are the CPU registers, you can view them in CCS by clicking View, Registers, CPU.
extern cregister volatile unsigned int CSR;     // declare CSR register 
extern cregister volatile unsigned int IER;     // declare IER register 
extern cregister volatile unsigned int ICR;     // declare ICR register 
//
//   NestISRContext -- Save context info for nested ISRs
//

////////////////////////////////////////////////////////



//---------------------------------------------------------------------------
//  IIMain() --  main function
//---------------------------------------------------------------------------
void IIMain()
{
	for( debug_nav_msg_index=0;debug_nav_msg_index<debug_nav_msg_len;debug_nav_msg_index++)
	{
		debug_nav_msg[debug_nav_msg_index] = 0;
	}
	debug_nav_msg_index = 0;

	freq_bin_index = 0;

	search_SVprn1to32 = 12; // start search with PRN x
	

	int temp = 0;
    cio << init;
    cio.At(Point(10, 0));
    cio << bold << "Peiqing GPS/GLONASS/Galileo/Beidou Satellite Receiver\n\n" << normal << endl; cio << noshowcursor << flush;
	sprintf(OPstring,"Peiqing GPS/GLONASS/Galileo/Beidou Satellite Receiver\n");
    my_OPfunc(OPstring, p_myIRQ);

/*	bool all_Reg_OK = true;
	int test;

	for(test=0; test<100; test++)
	{
		bool temp_all_Reg_OK;
		temp_all_Reg_OK = registerRW_Validation();
		if (temp_all_Reg_OK == false)
		{
			all_Reg_OK = false;
			break;
		}
	}
	if (all_Reg_OK == false)
		cio << bold << "Did you upload the FPGA? Control Registers is not accessible\n" << endl;
	else
		cio << bold << "FPGA Control Registers Validated\n" << endl;*/

	unsigned int * Ptr_IntType_regAddr = (unsigned int*) 0x80100000; // Interrupt type write - edge/level
	unsigned int * Ptr_IntPolarity_regAddr = (unsigned int*) 0x80110000; // Interrupt polarity write
	// rising edge trigger
	* Ptr_IntType_regAddr = 0; 
	* Ptr_IntPolarity_regAddr = ((1<<23)|(1<<22));
	// Dynamically create an Irq object tripped from ext interrupt 4
	Irq myIRQ( intEInt4 );
	p_myIRQ = & myIRQ;
	// Bind and install the interrupt vector 
	myIRQ.Install( Isr.Binder );
	int* ptr = (int*) mmInt4Enable; // DSP interrupt 4 source select
	*ptr = (1<<23); // fpdp_rx_fifo_int
	// Dynamically create an Irq object tripped from ext interrupt 5
	Irq my11msIRQ( intEInt5 );
	p_IRQ11ms = & my11msIRQ;
	// Bind and install the interrupt vector 
	my11msIRQ.Install( Isr11ms.Binder );
	ptr = (int*) mmInt5Enable; // DSP interrupt 4 source select
	*ptr = (1<<22); // fpdp_tx_fifo_int


	// Now the fpdp_rx_fifo interrupt will be routed to the DSP's external int 4.
	// Enable

//	cio << "\nCount the 2KHz IRQs...\n" << normal << endl; cio << noshowcursor << flush;
	
/*	DISABLEINTERRUPTS;
	while ((!cio.KbdHit())&&(temp <1))
	{
		cio << "unchanged Number of 2KHz IRQs as IRQ disabled: " <<  Isr.ISR2KHz_counter << " current display No." << temp << normal << endl; cio << noshowcursor << flush;
	 	Sleep(300);
		temp ++;
	}

	temp = 0;
    ENABLEINTERRUPTS;
	while ((!cio.KbdHit())&&(temp <1))
	{
		cio << "Changing dNumber of 2KHz IRQs as IRQ enabled: " <<  Isr.ISR2KHz_counter << " current display No." << temp << normal << endl; cio << noshowcursor << flush;
	 	Sleep(300);
		temp ++;
	}*/


	RANGE[GLONASS] = 20L;
    RANGE[GPS] = 2L;
	DPRAM_CHANNELS_ACTIVE_PQ = 0x0000;
	/* Setup data rates flag and the data rate */
    code_data_rate = 10;
    carr_data_rate = 10;
    CNR_data_rate  = 10;
    RatesChanged = FALSE;
 
    for(temp = 0;temp < NO_CHAN; temp++)
	{
        initialize_channel(temp);
    }
    error_type = NO_ERROR;  /* NO error has occured */

	initialize_carrier_IIR_coefficients();
	initialize_code_IIR_coefficients();
	initialize_timers();
	init_glo_parity_matrix();
	Generate_P_Code_Tables();

	int i;
    for(i=0;i<NO_CHAN;i++)
 	{
         Reset_Channel(i, GPS, p_myIRQ);
 	}

	reset_active_channels_matrix();

	initialize_scheduler();

     initialize_thresholds(1000L,GPS);
     initialize_thresholds(1000L,GLONASS);
     initialize_thresholds(1000L,INMARSAT);    /* JC 30/10/97 INM */
	// PQ added 080111
    // keep other bits unchanged and clear interrupt // PQ added 080112 
	*((unsigned int *)(CH_HARDWARE_BASE + WR_FLAGS)) =
       	((*((unsigned int *)(CH_HARDWARE_BASE + RD_FLAGS))& 0xF) | (INTPRIM|INTSEC));

	occur_Int = 0;
	occur_Int_with401 = 0;
    Set_Code_NCO(0,CACODE,code_freq_word[0][0], p_myIRQ);

    myIRQ.Enable(false);// PQ added 090706
    NOPDELAY();
    *((unsigned int *)(CH_HARDWARE_BASE + 0 * HARDWARE_OFFSET + WR_NCO3FCW_ACT)) = 0;
    NOPDELAY();
	myIRQ.Enable( true );

    //calc_noise(GLONASS, p_myIRQ);
    calc_noise(GPS, p_myIRQ);
	//Peiqing initialize_thresholds(SIGMA_2[GPS],INMARSAT);
	cio << "noise level is " << SIGMA_2[GPS] << normal << endl; cio << noshowcursor << flush;
     //initialize_thresholds(100L,GPS);
     //initialize_thresholds(100L,GLONASS);
     //initialize_thresholds(100L,INMARSAT);    /* JC 30/10/97 INM */

    initialize_scheduler();

    initialize_channel(0); // since Channel 0 is used in calc_noise, it needs to be reset.
    
    /*GB - before doing anything further sync to 1pps for IGEX campaign*/
    
    DISABLEINTERRUPTS;

    /* Asserts the address instructing a resync of 1PPS */
    /* Value is irrelevant */
	// PQ: comment it out because it has been only set to be 1, no other value
    // ONE_PPS_SYNC[0] = 1;

    /* Sets ISR_num to 1 */
    Process1pps(RESYNC_1PPS);

    /* Re-enables the 10PPS counter (should sync to 1PPS) */
    ENABLEINTERRUPTS;


    /* loop around for ever, initializing the channel when data is received */
    /* from the PC                                                          */

    // Reset the channel
    Reset_Channel(0,GPS,p_myIRQ);

    // Put the new satellite into the pending structure
    int ChnsToTrack = 0x01; 
    	// The bit/bits with value 1 corresponds to the HW ch/chs to be used for this tracking task
    	// bit 0-31 ~ Ch 0~31
	int TempChn; // used in for loop to go through all channels
	unsigned int Count;
    for(TempChn=0;TempChn<NO_CHAN;TempChn++)
    {
        if((ChnsToTrack >> TempChn) & 0x1) // current ch specified by TempChn needs to be used for tracking this SV
        {
            // If the channel is already running then stop. This allows
            // another task a chance at acquiring if there is one.     
            // Effectively, there is a 30s timeout of the channel.  
            if(tracking_state[TempChn] != UNUSED)
            {
                Stop_Integration(TempChn,PRIMARY,p_myIRQ);
                channel_state_log[TempChn] = 0;
                // Make a new count of 1ms tasks
                Count = 0;
                tasks.NumberOneMs = 0;
                tasks.NumberTwoMs = 0;
				
				// this while loop functionality is to be explained
                while((Count < NO_CHAN) && (active_channels[Count] != ACTIVE_CHANNELS_END))
                {
                    if(channel_state_log[active_channels[Count]] == 1)
                        tasks.NumberOneMs++;
                    if(channel_state_log[active_channels[Count]] == 2)
                        tasks.NumberTwoMs++;
                    Count++;
                }
            }

            // Reset the channel
            Reset_Channel(TempChn,GPS,p_myIRQ);
            initialize_channel(TempChn);

            // Put the new satellite into the pending structure, which will be used by 
			// Scheduler => AcquireChannel
            Pending[TempChn].Flag = NEW;
            Pending[TempChn].Doppler = 0;
            Pending[TempChn].System = GPS;
            Pending[TempChn].SV = (search_SVprn1to32-1)%32+1;
            Pending[TempChn].GPSONGLNFlag = FALSE;
        }
    }
	////////////////////////////////////////////////////////////////




    for(;;)
    {
		////////////////////////////////////////////////////////////////
		// PQ: added Code to replace the settings needs to be done in get_pc_commands().
        Scheduler(p_myIRQ);
		// PQ: comment it out temperarily, replaced by serial port comms later.
        // get_pc_commands();
    }

}

/*
void get_pc_commands(void)
{
    int doppler_int;
    int TempChn, ChnsToTrack, TempWord, ChnsToReset;
    int type, CodeBandWidth;
    int CodeOperation;
    unsigned int Count;
    int RatesSelection, TmpSelection;
    int ResetControl = 0;
	/////////////////////////////////////////////////////////////////////////////////////////
    if((DPRAM[0] & 0xFFFF) > 0x8000) // If the PC registers indicate SV to track
    {
        ChnsToTrack = (DPRAM[0] & 0x7fff);

        DPRAM[0] = 0x0000;

        for(TempChn=0;TempChn<NO_CHAN;TempChn++)
        {
            if((ChnsToTrack >> TempChn) & 0x1)
            {
                // If the channel is already running then stop.  This allows
                // another task a chance at acquiring if there is one.     
                // Effectively, there is a 30s timeout of the channel.  

                if(tracking_state[TempChn] != UNUSED)
                {
                    Stop_Integration(TempChn,PRIMARY);
                    channel_state_log[TempChn] = 0;

                    // Make a new count of 1ms tasks

                    Count = 0;
                    tasks.NumberOneMs = 0;
                    tasks.NumberTwoMs = 0;

                    while((Count < NO_CHAN) && (active_channels[Count] != ACTIVE_CHANNELS_END))
                    {
                        if(channel_state_log[active_channels[Count]] == 1)
                            tasks.NumberOneMs++;
                        if(channel_state_log[active_channels[Count]] == 2)
                            tasks.NumberTwoMs++;
                        Count++;
                    }
                }

                TempWord=DPRAM[OBS_1 + (2*TempChn)]&0xffff;
                system = TempWord & 0x1; // GPS or GLONASS
                satellite = (TempWord >> 1) & 0x1F; // SV#
                type = (TempWord >> 6) & 0xF; // a local variable but unrefered in this function
                CodeOperation = (TempWord >> 10) & 0x1; // a local variable but unrefered in this function
                CodeBandWidth = (TempWord >> 11) & 0x7; // a local variable but unrefered in this function
                doppler_int = DPRAM[OBS_2 + (2*TempChn)]&0xffff; // Doppler
                if(doppler_int & 0x8000)
				{
                    doppler_int -= 0x10000;
				}
                doppler = (int)(((float)doppler_int)/0.6F);

                // Reset the channel
                Reset_Channel(TempChn,system);
                initialize_channel(TempChn);

                // Put the new satellite into the pending structure
                Pending[TempChn].Flag = NEW;
                Pending[TempChn].Doppler = doppler;
                Pending[TempChn].System = system;
                Pending[TempChn].SV = satellite;
                Pending[TempChn].GPSONGLNFlag = FALSE;
            }
        }
    }

	/////////////////////////////////////////////////////////////////////////////////////////
    ResetControl = DPRAM[RESET_CONTROL];

    if((ResetControl & 0xFFFF) > 0)  // If PC tells receiver to reset a channel
    {
        ChnsToReset = (DPRAM[RESET_CONTROL] & 0x7FFF);

        // If a re-sync 1pps has occured
        if(ResetControl & 0x8000)
        {
            DISABLEINTERRUPTS;

            // Asserts the address instructing a resync of 1PPS
            // Value is irrelevant 
            ONE_PPS_SYNC[0] = 1;

            // Sets ISR_num to 1
            Process1pps(RESYNC_1PPS);

            // Re-enables the 10PPS counter (should sync to 1PPS)
            ENABLEINTERRUPTS;

            ChnsToReset = 0x7FFF;
        }


        for(TempChn=0;TempChn < NO_CHAN;TempChn++)
        {
            if((ChnsToReset >> TempChn) & 0x1)
            {
                if(tracking_state[TempChn] != UNUSED)  // Channel was running
                {
                    Reset_Channel(TempChn,system);
                    initialize_channel(TempChn);
                }

                Pending[TempChn].Flag = FALSE;  // Remove any pending lock
            }
        }

        DPRAM[CHANNELS_ACTIVE] &= ~ChnsToReset;
        DPRAM[RESET_CONTROL] = 0x0000;

        make_active_channels_matrix((int)DPRAM[CHANNELS_ACTIVE] & 0x7FFF);
    }

	/////////////////////////////////////////////////////////////////////////////////////////
    // Following section allows the variable data rate to be set up
    RatesSelection = DPRAM[DATA_RATE_CONTROL] & 0xFFFF;
    if(RatesSelection >= 0x8000) // New data rate
    {
        TmpSelection = RatesSelection & 0xF;
        new_code_data_rate = data_rates[TmpSelection];

        TmpSelection = (RatesSelection >> 4) & 0xF;
        new_carr_data_rate = data_rates[TmpSelection];

        TmpSelection = (RatesSelection >> 8) & 0xF;
        new_CNR_data_rate = data_rates[TmpSelection];

        RatesChanged = TRUE;          // Set flag for ISR to inspect
        RatesSelection &= ~(0x8000);  // Clear change bit

        DPRAM[DATA_RATE_CONTROL] = RatesSelection;
    }
*/


int glo_parity_matrix[77]; /* GLONASS parity matrix */



void disable_IRQ_func()
{
	p_myIRQ->Enable( false );
}





void initialize_channel(int i) // set software channel status variables, no register operations.
{
    int j;
    for(j = 0; j<3;j++)
    {
        Isqr_Qsqr_A[j][i] = 0;
        Isqr_Qsqr_B[j][i] = 0;
        I_A[j][i]         = 0;
        I_B[j][i]         = 0;
        Q_A[j][i]         = 0;
        Q_B[j][i]         = 0; 
        
        carrier_phi[j][i] = 0.0F;

        code_filter_state_1[j][i] = 0.0F;
        code_lock_thres[j][i] = 0;
    }
        
    track_L2_time[i] = 0;
    track_L1_time[i] = 0;
    P_code_timeout[i] = 0;
    p_filter_av[L1][i] = 0.0F;
    p_filter_av[L2][i] = 0.0F;
    carr_filter_state_1_float[L1][i] = 0.0F;
    carr_filter_state_2_float[L1][i] = 0.0F;
    carr_filter_state_1_float[L2][i] = 0.0F;
    carr_filter_state_2_float[L2][i] = 0.0F;
    
    carr_filter_state_1_long[L1][i] = 0L;
    carr_filter_state_1_long[L2][i] = 0L;
                  
    code_filter_sum[CA_primary][i] = 0.0F;
    code_filter_sum[P_L1][i] = 0.0F;
    code_filter_sum[P_L2][i] = 0.0F;
    
    code_filter_state_2[CA_primary][i] = 0.0F;
    code_filter_state_2[P_L1][i] = 0.0F;
    code_filter_state_2[P_L2][i] = 0.0F;
                  
    carr_filter_sum_long[i]  = 0;
    carr_filter_sum_float[i] = 0.0F;

    
    carrier_top[L1][i] = 0L;
    carrier_top[L2][i] = 0L;
    carrier_top_start[i] = 0L;
    carrier_top_start[i] = 0L;
    
    bit_counter[i] = 0;
    
    num_valid_bits[i] = 0;
    sign_store[i] = 0;

    freq_offset[i] = 0;
    freq_offset_top[i] = 0;

    carrier_z[0][i]  = 0.0F;
    carrier_z[1][i]  = 0.0F;

    tracking_state[i] = UNUSED;
    task_counter[i] = 0;
    channel_num[i] = 0;
    freq_err_out[i] = 0.0F;
    code_loop_gain[i] = 10.0F;


    codeless_bandwidth[i] = ONE_POINT_0;

    loop_gain[i] = 10.0F;


    parity_check[i] = 0;
    bit_count[i] = 0;
    data_store[i] = 0;
    sys_time[i] = 0;

    phi_acc[i] = 0.0F;
    w[i] = 0.0F;
    x[i] = 0.0F;
    spacing[i] = ONE_CHIP;
    filtered_carr_error[L1][i] = 45.0F/360.0F; // PQ 20090711 added /360, because the unit should be cycles
    filtered_carr_error[L2][i] = 45.0F/360.0F; // PQ 20090711 added /360, because the unit should be cycle
    bit_sync[i] = FALSE;
    MS_data_store[i] = 0xffffffff;
    interrupt_number[i] = 0;
    Integration_time[i] = 0.001F;
    
    shift_p[i] = FIRST;
    p_chips_added[i] = 0;
    code_phase_store[i] = 0.0F;
    carr_phase_store[i] = 0.0F;

    CNR_sum[0][i] = 0;
    CNR_sum[1][i] = 0;
    CNR_sum[2][i] = 0;
    CNR_sum_count[0][i] = 0;
    CNR_sum_count[1][i] = 0;
    CNR_sum_count[2][i] = 0;

    CODELESS_TRACKING[i] = FALSE;

    I_A_INT[L2][i] = 0L;
    I_B_INT[L2][i] = 0L;
    Q_A_INT[L2][i] = 0L;
    Q_B_INT[L2][i] = 0L;

    codeless_code_aid_freq[L1][i] = 0.0F;
    codeless_code_aid_freq[L2][i] = 0.0F;

    codeless_carrier_aid_freq[i] = 0.0F;

    p_search_int_count[i] = 0;

    re_acquire_p[i] = FALSE;

    channel_state_log[i] = 0;
}

void initialize_carrier_IIR_coefficients(void)
{   
	///////////////////////////////////////////////////
	// PQ: coefficients to be checked!
	///////////////////////////////////////////////////
    float T,k1,k2,k3;


    k1 =   0.023082F;
    k2 =   0.269499F;
    k3 =   1.573341F;
    T = 0.001;

    CARRIER_IIR_B[0] = 2.0F*k1/T + 4.0F*k2 + 4.0F*k3*T;
    CARRIER_IIR_B[1] = 4.0F*k1/T + 6.0F*k2 + 3.0F*k3*T;
    CARRIER_IIR_B[2] = 2.0F*k1/T + 2.0F*k2 +      k3*T;
    
    CARRIER_IIR_A[0] =       k3*T*T/2.0F - 0.1F +      k1 +      k2*T;
    CARRIER_IIR_A[1] = 0.05F*k3*T*T      + 0.9F + 0.1F*k1 + 0.1F*k2*T;
    CARRIER_IIR_A[2] = 0.45F*k3*T*T             + 0.9F*k1 + 0.9F*k2*T;
}                                    

void initialize_code_IIR_coefficients(void)
{
	///////////////////////////////////////////////////
	// PQ: coefficients to be checked!
	///////////////////////////////////////////////////
    float T,k1,k2;

    T = 0.001F;

    /* 1Hz filter */
    k1 = 0.0021F;
    k2 = 0.0039F;

    c0 = 2.0F*k1/T + 4.0F*k2;
    c1 = 2.0F*k1/T + 6.0F*k2;
    c2 = 2.0F*k2;

    r1 = k2*T + k1 + 2.9F;
    r2 = 1.9F*(1.0F + k1 + k2*T);
}

void initialize_timers(void)
{
	///////////////////////////////////////////////////
	// PQ: function to be completed.
	///////////////////////////////////////////////////
    timer_one_pps++;
    timer_ms = 0;
}

void init_glo_parity_matrix(void)
{
	///////////////////////////////////////////////////
	// PQ: function to be checked with Ian Kitching's PhD thesis or the GLONASS ICD.
	///////////////////////////////////////////////////
    int row;
    int value;
    int nvalue = 64;
    int parity;
    int shift;
   
    /* loop around 83 to 1 missing out powers of 2 */    
    for(value = 83, row = 1 ; value != 0 ; value--)
    {
        if(value != nvalue)
        {
            for(shift = 1,parity = 128;shift != 128;shift <<= 1)
            {
                /* Effecively count the number of ones to add overall */
                /* odd parity                                         */
                
                if((value & shift) == shift)
                    parity += 128;            
            }                    
            
            /* Update the parity matrix */
            glo_parity_matrix[row] = value | (parity & 128);
            row++;
        }
        else
            nvalue >>= 1;
    }
}


void initialize_thresholds(long variance,int sys)
{

    SIGMA_2[sys] = variance;
    //STATE_1_THRES[sys] = (10L*SIGMA_2[sys]);
/*GB MR says drop this to 7*/
    //STATE_1_THRES[sys] = (7L*SIGMA_2[sys]);
    //STATE_1_THRES[sys] = (10L*SIGMA_2[sys]);
    STATE_1_THRES[sys] = (20L*SIGMA_2[sys]); // PQ 090714 

    STATE_2_THRES[sys] = (82L*SIGMA_2[sys]); // WP2112 Page28

    STATE_3_THRES[sys] = (240L*SIGMA_2[sys]);

    STATE_6_THRES[sys] = (44L*SIGMA_2[sys]);


    STATE_7CA_THRES[sys] = (800L*SIGMA_2[sys]);

    STATE_7_CA_THRES[sys] = (2000L*SIGMA_2[sys]);
    STATE_7_P1_THRES[sys] = (2000L*SIGMA_2[sys]);
    STATE_7_P2_THRES[sys] = (2000L*SIGMA_2[sys]);

    STATE_7_THRES[sys] = (300L*SIGMA_2[sys]);

    STATE_7_P1_THRES_CODELESS[sys] = ((long)(2300>>4)*SIGMA_2[sys]);
    STATE_7_P2_THRES_CODELESS[sys] = ((long)(2300>>4)*SIGMA_2[sys]);

}


int calc_noise(int sys, Irq *p_myIRQ)
{
    long sum;
    long df;
    //long counter; 
    int counter; // PQ 0109
    int i,j;
    float sig_2;


    long temp_store[NUM_SUMS];  /* Temp store for accumulated I2+Q2 */
    int temp_index = 0;         /* Index to temp_store */

    for(j = 0; j<NUM_SUMS; j++)
    {
        counter = task_counter[0];

        /* Need to load this channel into the pending structure to run */
        if(sys == GPS)
        {
            Pending[0].Flag = NEW;
            Pending[0].Doppler = 0;           /* Zero doppler */
            Pending[0].System = GPS;
            Pending[0].SV = 32;                /* PRN 32 */
            Pending[0].GPSONGLNFlag = FALSE;  /* GPS_ON_GLN FALSE */
        }
        else
        {
            Pending[0].Flag = NEW;
            Pending[0].Doppler = 0;            /* Zero doppler */
            Pending[0].System = GLONASS;
            Pending[0].SV = 12;                /* Channel 12 */
            Pending[0].GPSONGLNFlag = TRUE;    /* GPS_ON_GLN TRUE */
        }

        sum = 0;
        i = 0;
        do
        {
            Scheduler(p_myIRQ);
            if( counter != task_counter[0] )
            {
                counter = task_counter[0];

                df =   I_B[CA_primary][0]*I_B[CA_primary][0]
                     + Q_B[CA_primary][0]*Q_B[CA_primary][0];

				debug_df[j*1000+i] = df; // df approximately 100
				debug_I_B[j*1000+i] = I_B[CA_primary][0];
				debug_Q_B[j*1000+i] = Q_B[CA_primary][0];

                sum += df;
                i++;
            }
        }
        while(i<1000);
        sig_2 = (float)sum/(2.0F*1000.0F);

        temp_store[temp_index] = (long)(sig_2*1000.0F);
		temp_index++;
    }

    /* Clear Channel ready for normal operation */
    Reset_Channel(0,GPS,p_myIRQ);

    sum = 0;
    for(j = 0;j<NUM_SUMS;j++)
    {
        sum += (float)temp_store[j]/1000.0F;
    }
    sum/=(float)NUM_SUMS;

//    if(sys == GLONASS) // PQ comment out since I don't know why GLN noise floor doesn't need to be estimated.
//        sum = 60;
//	sum *= 2;
    initialize_thresholds((long)sum,sys);
	return -1;
}


void return_to_one(int chn, Irq *p_myIRQ)
{
	inverted = 0;// PQ 090722 added.
	CodeSearchTime = 0;
	last_code_energy_presence = 0;

	////////////////////
	index_debug_FLL_code_lock = 0;

	for( debug_filtered_carr_error_pnt=0;debug_filtered_carr_error_pnt<200;debug_filtered_carr_error_pnt++)
	{
		debug_filtered_carr_error[debug_filtered_carr_error_pnt] = 0.0f;
		debug_phase_error1[debug_filtered_carr_error_pnt] = 0.0f;
	}
	debug_filtered_carr_error_pnt = 0;

	/////////////////////
    int Count;


    /* Place a flag into the Pending structure that indicated the channel */
    /* is awaiting relock.                                                */

    /*Pending[chn].Flag = NEW;*/
    switch(tracking_state[chn])
    {

        case UNUSED:
        case STATE_0:
			break;
        case STATE_1:
			Pending[chn].Flag = NEXT_FREQ_BIN;
			freq_bin_index ++;
			if (freq_bin_index == freq_bin_NO)
				freq_bin_index = 0;
            break; 
        case STATE_2:
			Pending[chn].Flag = NEXT_FREQ_BIN;
			freq_bin_index ++;
			if (freq_bin_index == freq_bin_NO)
				freq_bin_index = 0;
            break; 
        case STATE_3:
			Pending[chn].Flag = NEXT_FREQ_BIN;
			freq_bin_index ++;
			if (freq_bin_index == freq_bin_NO)
				freq_bin_index = 0;
            break; 
        case STATE_4 :
			Pending[chn].Flag = NEXT_FREQ_BIN;
			freq_bin_index ++;
			if (freq_bin_index == freq_bin_NO)
				freq_bin_index = 0;
            break; 
        case STATE_5 :
        case STATE_5A:  
        case STATE_5B:
        case STATE_5C:
        case STATE_5CA:
        case STATE_5D:
        case STATE_5E:
        case STATE_5F:
        case STATE_6:   
        case STATE_7:
        case STATE_7A:
        case STATE_7B:
        case STATE_7C:
        case STATE_7D:
        case STATE_7E:
        case STATE_7F:
			Pending[chn].Flag = RELOCK;
            break;
        default:
            error_type = INVALID_TRACK_STATE;
            break;
    }

    tracking_state[chn] = UNUSED;

    CA_Lag_Mode[chn] = WIDE_CA+W_EARLY+W_2_SPACING;
    Set_Lag_Spacing(chn,PRIMARY,CA_Lag_Mode[chn], p_myIRQ);
    Set_Lag_Spacing(chn,SECONDARY,CA_Lag_Mode[chn], p_myIRQ);
    Shift_Code_NCO_Phase (chn,CACODE,255,ASYNC,FALSE, p_myIRQ);

    Integration_time[chn] = 0.001F;

    freq_err_out[chn] = 0.0F;
    code_loop_gain[chn] = 10.0F;

    carr_filter_state_1_float[L1][chn] = 0.0F;
    carr_filter_state_2_float[L1][chn] = 0.0F;
    carr_filter_state_1_float[L2][chn] = 0.0F;
    carr_filter_state_2_float[L2][chn] = 0.0F;

    code_filter_state_1[CA_primary][chn] = 0.0F;
    code_filter_state_1[P_L1][chn] = 0.0F;
    code_filter_state_1[P_L2][chn] = 0.0F;

    carr_filter_state_1_long[L1][chn] = 0;
    carr_filter_state_1_long[L2][chn] = 0;
                  
    code_filter_sum[CA_primary][chn] = 0.0F;
    code_filter_sum[P_L1][chn] = 0.0F;
    code_filter_sum[P_L2][chn] = 0.0F;
    
    code_filter_state_2[CA_primary][chn] = 0.0F;
    code_filter_state_2[P_L1][chn] = 0.0F;
    code_filter_state_2[P_L2][chn] = 0.0F;
                  
    carr_filter_sum_long[chn]  = 0;
    carr_filter_sum_float[chn] = 0.0F;
    
    loop_gain[chn] = 10.0F;

    parity_check[chn] = 0;
    
    /* Need to check difference between bit_count & bit_counter & clarify names */
    bit_count[chn] = 0;
    bit_counter[chn] = 0;
    
    num_valid_bits[chn] = 0;
    sign_store[chn] = 0;

    freq_offset[chn] = 0;
    freq_offset_top[chn] = 0;

    data_store[chn] = 0;
    sys_time[chn] = 0;

    phi_acc[chn] = 0.0F;
    w[chn] = 0.0F;
    x[chn] = 0.0F;
    spacing[chn] = ONE_CHIP;
    filtered_carr_error[L1][chn] = 45.0F/360.0F; // PQ 20090711 added /360, because the unit should be cycle
    filtered_carr_error[L2][chn] = 45.0F/360.0F; // PQ 20090711 added /360, because the unit should be cycle
    code_lock_thres[CA_primary][chn] = 0;
    task_counter[chn] = 0;    
    bit_sync[chn] = FALSE;
    MS_data_store[chn] = 0xffffffff;
    carrier_phi[0][chn] = 0.0F;
    carrier_phi[1][chn] = 0.0F;
    carrier_phi[2][chn] = 0.0F;

    carrier_z[0][chn] = 0.0F;
    carrier_z[1][chn] = 0.0F;
    interrupt_number[chn] = 0;
    
    shift_p[chn] = FIRST;
    p_chips_added[chn] = 0;
    code_phase_store[chn] = 0.0F;
    carr_phase_store[chn] = 0.0F;

    /* Reset the current estimates of code and carrier freq */
    code_freq_word[L1][chn]    = start_code_freq[chn];
    carrier_low[L1][chn] = carrier_low_start[chn];
    carrier_top[L1][chn] = carrier_top_start[chn];

    /* Reset the hardware to the a priori estimate of */
    /* code and carrier frequency                     */
    Set_Code_NCO(chn,CACODE,code_freq_word[0][chn], p_myIRQ);
    // Set_Carrier_NCO (chn,L1CARRIER,carrier_t op_start[L1][chn],carrier_f req_word[0][chn], p_myIRQ);
    // PQ: make this change since carrier_ top[L1][chn] is set to be carrier_top _start[L1][chn]
    Set_Carrier_NCO (chn,L1CARRIER,carrier_top[L1][chn],carrier_low[L1][chn], p_myIRQ);


    /********************************************************************/
    /*                                                                  */
    /* With the scheduler in place, should a channel unlock for any     */
    /* reason, the channel must be removed from the picture and await   */
    /* its turn to re-acquire.  If all channels should unlock, this will*/
    /* ensure that they can all finally relock.                         */
    /*                                                                  */
    /* The apriori estimate for the channel will therefore not be       */
    /* exactly accurate, but it is only an estimate anyway and better   */
    /* than nothing.                                                    */
    /*                                                                  */
    /********************************************************************/

    /* Stop the integration timer will stop the channel interrupting and */
    /* halt all activity.  Currently, no more setting-up will be done.   */
    /* It is assumed that when the integration timer is restarted, the   */
    /* channel will restart as it stopped.                               */

    Stop_Integration(chn,PRIMARY, p_myIRQ);
    /*Reset_Channel(chn,GPS);*/

    /* Notify scheduler that the channel is no longer running */

    channel_state_log[chn] = 0;

    /* And make a new count of 1ms tasks */

    Count = 0;
    tasks.NumberOneMs = 0;
    tasks.NumberTwoMs = 0;

    while((Count < NO_CHAN) && (active_channels[Count] != ACTIVE_CHANNELS_END))
    {
        if(channel_state_log[active_channels[Count]] == 1)
            tasks.NumberOneMs++;
        if(channel_state_log[active_channels[Count]] == 2)
            tasks.NumberTwoMs++;
        Count++;
    }
}

bool registerRW_Validation(void)
{
	bool all_Reg_OK = true;
	unsigned int RWvalue;
	unsigned int readbackValue;
	/////////////////////////////////////////////////////////////
	// These three arrays need to be updated together with #define No_Reg_Tested
	/////////////////////////////////////////////////////////////
//	char registername[No_Reg_Tested][6] = {"FLAGS",
//		"CTRL1",
//		"CTRL2",
//		"CTRL3",
//		"CTRL4"};
	unsigned int* ptr[No_Reg_Tested] = {(unsigned int *)(CH_HARDWARE_BASE + WR_FLAGS),
		(unsigned int *)(CH_HARDWARE_BASE + WR_CTRL1),
		(unsigned int *)(CH_HARDWARE_BASE + WR_CTRL2),
		(unsigned int *)(CH_HARDWARE_BASE + WR_CTRL3),
		(unsigned int *)(CH_HARDWARE_BASE + WR_CTRL4)};//,
		//(unsigned int *)(CH_HARDWARE_BASE + WR_ACC1I)};
	unsigned int mask[No_Reg_Tested] = {0x0F,
		0x3F,
		0x3F,
		0x0F,
		0x03}; //,
		// 0x07FFFFF}; // pick the biggest one and place it in "for(RWvalue=0; RWvalue<="
	/////////////////////////////////////////////////////////////

	int channelNo = 0;
	int index_Reg;
	unsigned int* ptr_this;
	for (channelNo = 0; channelNo<NO_CHAN; channelNo++)
	{
		for(index_Reg = 0; index_Reg<No_Reg_Tested; index_Reg++)
		{
			ptr_this = ptr[index_Reg] + channelNo * HARDWARE_OFFSET;
			for(RWvalue=0; RWvalue<=mask[index_Reg]; RWvalue++)
			{
				*ptr_this = RWvalue;
				NOPDELAY();
				readbackValue = *ptr_this;
				readbackValue &= mask[index_Reg];
				if (readbackValue != RWvalue)
				{
					all_Reg_OK = false;
				}
			}
			*ptr_this = 0;
		}
	}
	return all_Reg_OK;
}

/*	int* ptr = (int*) 0x80210200;
	for( int n=1 ; n < 5 ; n++ )
	{
		 *ptr = n; // write to logic
		int tt;
            int TrueVal = n;
            tt = *ptr; // read from logic
            if (tt == TrueVal)
                    cio << "Correct" <<  tt << "\n";
            else
                    cio << "Return value is" << tt <<",Correct value should be" << n << "\n";
    }

*/





// PQ moved from lock.cpp



// PQ: DPRAM should be replaced
// extern int DPRAM_CHANNELS_ACTIVE_PQ; // PQ, 080115 change from "extern long DPRAM_CHANNELS_ACTIVE_PQ"
// extern int glo_parity_matrix[77]; /* GLONASS parity matrix */


void track(int chn, Irq *p_myIRQ)
{
    switch(tracking_state[chn])
    {

        case UNUSED:
            break;

        case STATE_0:
            tracking_state[chn] = STATE_1;
            break;

        case STATE_1:
	        track_code_search(chn, p_myIRQ);
	        break;
            
        case STATE_2:
            track_fll_pull_in(chn, p_myIRQ);
            break; 

        case STATE_3:
            track_pll_pull_in(chn, p_myIRQ);
            break; 

        case STATE_4 :
            detect_edges(chn, p_myIRQ);
            break;
        
        case STATE_5 :
        case STATE_5A:  
        case STATE_5B:
        case STATE_5C:
        case STATE_5CA:
        case STATE_5D:
        case STATE_5E:
        case STATE_5F:
            change_integration_period(chn, p_myIRQ);
            break;
                            
        case STATE_6:   
            steady_state_long_integration_CA_tracking(chn, p_myIRQ);
            break;
                
        case STATE_7:
        case STATE_7A:
        case STATE_7B:
        case STATE_7C:
        case STATE_7D:
        case STATE_7E:
        case STATE_7F:
            P_code_tracking(chn, p_myIRQ);             
            break;
        default:
            error_type = INVALID_TRACK_STATE;
            break;
    }
}

void track_code_search(int chn, Irq *p_myIRQ)
{                                                                           

    read_correlator_totals(chn,CA_primary,NORM);
	// The next two read_correlator_totals are not used because GB says:
	// GB only CA accumulators are reliable - hence only use them
    // read_correlator_totals(chn,P_L1,NORM);
    // read_correlator_totals(chn,P_L2,NORM);

    /* Form I^2 + Q^2 for the CA_primary sub-channel                */
    /* NOTE:- Each sub-channel has two correlators (A & B) in I & Q */                      
    // For now, the local replca code is 
    Isqr_Qsqr_A[CA_primary][chn] = (I_A[CA_primary][chn] * I_A[CA_primary][chn]) +
                                   (Q_A[CA_primary][chn] * Q_A[CA_primary][chn]);
                                    
    Isqr_Qsqr_B[CA_primary][chn] = (I_B[CA_primary][chn] * I_B[CA_primary][chn]) +
                                   (Q_B[CA_primary][chn] * Q_B[CA_primary][chn]);

    /* Form I^2 + Q^2 for the P_L1 & P_L2 sub-channel           */
    /* NOTE - during acquisition, the L1 and L2 sub-channel     */
    /*        correlators are used as additional C/A correlators*/
    /*        to reduce the acquisition time                    */

    /* P L1 */

    Isqr_Qsqr_A[P_L1][chn] = (I_A[P_L1][chn] * I_A[P_L1][chn]) +
                             (Q_A[P_L1][chn] * Q_A[P_L1][chn]);

    Isqr_Qsqr_B[P_L1][chn] = (I_B[P_L1][chn] * I_B[P_L1][chn]) +
                             (Q_B[P_L1][chn] * Q_B[P_L1][chn]);

    /* P L2 */

    Isqr_Qsqr_A[P_L2][chn] = (I_A[P_L2][chn] * I_A[P_L2][chn]) +
                             (Q_A[P_L2][chn] * Q_A[P_L2][chn]);

    Isqr_Qsqr_B[P_L2][chn] = (I_B[P_L2][chn] * I_B[P_L2][chn]) +
                             (Q_B[P_L2][chn] * Q_B[P_L2][chn]);

    /* Now check for energy */
/*
    if((Isqr_Qsqr_A[CA_primary][chn] > STATE_1_THRES[sys[chn]]) ||
       (Isqr_Qsqr_A[P_L1][chn]       > STATE_1_THRES[sys[chn]]) ||
       (Isqr_Qsqr_A[P_L2][chn]       > STATE_1_THRES[sys[chn]]) ||
       (Isqr_Qsqr_B[CA_primary][chn] > STATE_1_THRES[sys[chn]]) ||
       (Isqr_Qsqr_B[P_L1][chn]       > STATE_1_THRES[sys[chn]]) ||
       (Isqr_Qsqr_B[P_L2][chn]       > STATE_1_THRES[sys[chn]]))
*/
/*GB only CA accumulators are reliable - hence only use them*/
// PQ: why GB got this concern
    task_counter[chn]++;

    if( (Isqr_Qsqr_A[CA_primary][chn] > STATE_1_THRES[sys[chn]]) ||
        (Isqr_Qsqr_B[CA_primary][chn] > STATE_1_THRES[sys[chn]]) )
    { 
    	// Isqr_Qsqr_A[0][0] Isqr_Qsqr_B[0][0]
		// cio << "track_code_search pass with" << Isqr_Qsqr_A[CA_primary][chn] << " or " << Isqr_Qsqr_B[CA_primary][chn] << " > " << STATE_1_THRES[sys[chn]] << "\n" << endl;
		last_code_energy_presence = 1;
		
		if (CodeSearchTime == 0)
		{
			if (Isqr_Qsqr_A[CA_primary][chn] > Isqr_Qsqr_B[CA_primary][chn])
				CodeSearch_squ_result[CodeSearchTime] = Isqr_Qsqr_A[CA_primary][chn];
			else 
				CodeSearch_squ_result[CodeSearchTime] = Isqr_Qsqr_B[CA_primary][chn];
			CodeSearchTime++;
		}
		else
		{
			if (last_code_energy_presence == 1)
			{
				if (Isqr_Qsqr_A[CA_primary][chn] > Isqr_Qsqr_B[CA_primary][chn])
					CodeSearch_squ_result[CodeSearchTime] = Isqr_Qsqr_A[CA_primary][chn];
				else 
					CodeSearch_squ_result[CodeSearchTime] = Isqr_Qsqr_B[CA_primary][chn];

				CodeSearchTime++;
			}
			else
				CodeSearchTime = 0;
		}

		if (CodeSearchTime == CODE_SEARCH_CONFIRM)
		{
	        CA_Lag_Mode[chn] = WIDE_CA + W_EARLY + W_1_SPACING; /* 1 chip */
	        Set_Lag_Spacing(chn,PRIMARY,CA_Lag_Mode[chn], p_myIRQ);
	        Set_Lag_Spacing(chn,SECONDARY,CA_Lag_Mode[chn], p_myIRQ);

	    /* Calculate the correlator with the maximum energy and shift the code accordingly */
	        find_maximum_energy(chn, p_myIRQ);

	        spacing[chn] = ONE_CHIP;

	        task_counter[chn] = 0L;

	    /* The threshold is exceeded therefore go to STATE_2 */
	        if(sys[chn] == INMARSAT)  /* JC 30/10/97 INM - miss out FLL for INM */
                tracking_state[chn] = STATE_3;
            else
                tracking_state[chn] = STATE_2;
		}
		// else: we neither start a new search, nor shift Code position. Stick to the current position to confirm the Code energy.
    }
    else if(task_counter[chn] == 1023) // advance 2 Code Chip once, go through 1023 Code chips for twice
	{
	    return_to_one(chn, p_myIRQ);
	}
	else
    {   
	last_code_energy_presence = 0;

    Shift_Code_NCO_Phase (chn,CACODE,128,ASYNC,FALSE, p_myIRQ); /* advance code by 2 chips */
    }
}

void find_maximum_energy(int chn, Irq *p_myIRQ)
{
    int int_phase_offset;
    float err = 0.0F;
    // long max;           /* maximum sum(I^2+Q^2) value */
    int index;          /* Array index with maximum value */
    long energy[6];


    energy[0] = Isqr_Qsqr_A[CA_primary][chn];
    energy[1] = Isqr_Qsqr_B[CA_primary][chn];

/*GB - only using first two correlators
    energy[2] = Isqr_Qsqr_A[P_L1][chn];
    energy[3] = Isqr_Qsqr_B[P_L1][chn];
    energy[4] = Isqr_Qsqr_A[P_L2][chn];
    energy[5] = Isqr_Qsqr_B[P_L2][chn];
*/
    // max = energy[0];
    index = 0;

/* GB
    for(i = 1;i < 6;i++)
    {
        if(energy[i] > max)
        {
            max = energy[i];
            index = i;
        }
    }
*/
    if(energy[1]>energy[0])
	{
    	index=1;
	}
	// index = 0: int_phase_offset = +8
	// index = 1: int_phase_offset = -8
    int_phase_offset = -(16*index) + 8;

/*GB added the following*/
	// PQ 090716
	// 		energy[0] - energy[1]
	// 12 * ----------------------- -4 
	// 		energy[0] + energy[1]
	// 12 is the gain coefficient, it is calculated by ???
 	// 4 is the mid point of 8 used in the step if(energy[1]>energy[0])
    if(index==0)
    {
        err = (12.0F*(((float)(energy[0] - energy[1])) / ((float)(energy[0] + energy[1]))) - 4.0F);
    }
    else
    {
        err = (12.0F*(((float)(energy[0] - energy[1])) / ((float)(energy[0] + energy[1]))) + 4.0F);
    }

/*GB commented out the following*/
/*
    if((index >= 1) && (index <= 4))
    {
        / Middle correlator can create a good estimate of phase error /

        err = 8.0F*((float)(energy[index-1] - energy[index+1])) / ((float)energy[index]);
    }

    if(index == 0)
    {
        err = (8.0F*(((float)(energy[0] - energy[1])) / ((float)(energy[0] + energy[1]))) - 8.0F);
    }


    if(index == 5)
    {
        err = (8.0F*(((float)(energy[4] - energy[5])) / ((float)(energy[4] + energy[5]))) - 40.0F);
    }
*/
    /* Avoid rounding errors when converted to integer */
    if(err < 0.0F)
        err -= 0.5F;
    else
        err += 0.5F;

    int_phase_offset += (int)err;

    I_punct_previous[CA_primary][chn] = I_B[CA_primary][chn];
    Q_punct_previous[CA_primary][chn] = Q_B[CA_primary][chn];

    if(int_phase_offset!=0.0F)
        Shift_Code_NCO_Phase (chn,CACODE,int_phase_offset*4,ASYNC,FALSE, p_myIRQ);
}


void track_fll_pull_in(int chn, Irq *p_myIRQ)
{
    float phase_offset;
    float freq_err;
    long denom;
    float conv;

    read_correlator_totals(chn,CA_primary,NORM);
    read_correlator_totals(chn,P_L1,NORM);

    task_counter[chn]++;

    // Frequency lock-loop WP2112 Page22 Eq 41
    freq_err  = (float) (-1*(   I_B[CA_primary][chn]*Q_punct_previous[CA_primary][chn]
                              - Q_B[CA_primary][chn]*I_punct_previous[CA_primary][chn]));

    freq_err /= 0.004*PI*(float)(   I_B[CA_primary][chn]*I_B[CA_primary][chn]
                                  + Q_B[CA_primary][chn]*Q_B[CA_primary][chn]
                                  + I_punct_previous[CA_primary][chn]*I_punct_previous[CA_primary][chn]
                                  + Q_punct_previous[CA_primary][chn]*Q_punct_previous[CA_primary][chn]);

    //freq_err *= -1; // remove it !!!!!!!!!!!!
    // Filter the discriminator output (200Hz B/W) WP2112 Page22 Eq 42
    freq_err_out[chn] = freq_err_out[chn]*0.4286F + (1.0F - 0.4286F)*freq_err;


    // Update the carrier frequency word
    conv = freq_err_out[chn] * ONE_HZ_WORD_CARR;

    // Account for incorrect float to long rounding
    if(conv < 0.0F)
	{
        conv -= 0.5F;
	}
    else
	{
        conv += 0.5F;
	}
    carrier_low[L1][chn] += (long)conv;

    // Now need to determine whether the bottom word of the NCO control will overflow 
    // and it is necessary to update the top word                                     
    carrier_word_wrap(chn,L1,FALSE);

    // Apply the new frequency to H/W
    Set_Carrier_NCO(chn,L1CARRIER,carrier_top[L1][chn],carrier_low[0][chn], p_myIRQ);

	// save current I/Q_p for next epoch FLL computation purpose
    I_punct_previous[CA_primary][chn] = I_B[CA_primary][chn];
    Q_punct_previous[CA_primary][chn] = Q_B[CA_primary][chn];

    // I^2 + Q^2 (punctual)
	long temp_code_lock = I_B[CA_primary][chn]*I_B[CA_primary][chn]
                         + Q_B[CA_primary][chn]*Q_B[CA_primary][chn];
    code_lock_thres[CA_primary][chn] += temp_code_lock;

	debug_FLL_code_lock[index_debug_FLL_code_lock] = temp_code_lock;
	debug_freq_err_out[index_debug_FLL_code_lock] = freq_err_out[chn];
	debug_freq_err[index_debug_FLL_code_lock] =	freq_err;
	index_debug_FLL_code_lock ++;
	if (index_debug_FLL_code_lock == FLL_ms_Num)
		index_debug_FLL_code_lock = 0;

    // approximation of Eq.33 in WP2112 Page18 
    // CA_primary contains Early, P_L1 contains Late
    denom =   I_A[CA_primary][chn]*I_A[CA_primary][chn]
            + Q_A[CA_primary][chn]*Q_A[CA_primary][chn]
            + I_A[P_L1][chn]*I_A[P_L1][chn]
            + Q_A[P_L1][chn]*Q_A[P_L1][chn];
    if( denom != 0 )
    {
        // Code Phase Error Estimate
        phase_offset  = discriminator_gain[spacing[chn]]
                        * (float)(I_A[CA_primary][chn] * I_A[CA_primary][chn]
                               +  Q_A[CA_primary][chn] * Q_A[CA_primary][chn]
                               -  I_A[P_L1][chn] * I_A[P_L1][chn]
                               -  Q_A[P_L1][chn] * Q_A[P_L1][chn]);

        phase_offset /= (float)denom;
        
        if(phase_offset > 8.0F)
            phase_offset = 8.0F;

        if(phase_offset < -8.0F)
            phase_offset = -8.0F;
    }
    else
	{
        phase_offset = 0.0F;
    }                
    code_second_order(chn,code_loop_gain[chn]*phase_offset/16.0F, p_myIRQ);

    // Run the FLL for 20ms and then switch on the DPLL
    if(task_counter[chn] == FLL_ms_Num)
    {
        if(code_lock_thres[CA_primary][chn] > STATE_2_THRES[sys[chn]])
		{ //code_lock_thres[0][chn]
			cio << "FLL Pass " << freq_bin[freq_bin_index] << "(Hz) " << code_lock_thres[CA_primary][chn] << ">" << STATE_2_THRES[sys[chn]] << normal << endl; cio << noshowcursor << flush;
            tracking_state[chn] = STATE_3;
		}
        else
        {
			// cio << "FLL Fail " << freq_bin[freq_bin_index] << "(Hz) " << code_lock_thres[CA_primary][chn] << normal << endl; cio << noshowcursor << flush;
            return_to_one(chn, p_myIRQ);
        }
        code_lock_thres[CA_primary][chn] = 0;
        task_counter[chn] = 0;
    }
}

void track_pll_pull_in(int chn, Irq *p_myIRQ)
{
    float phase_error;
    int sign;

    float phase_offset;
    long denom;



    if(task_counter[chn] < CHANGE_CORRELATORS_EL)
    {
        read_correlator_totals(chn,CA_primary,NORM);
        read_correlator_totals(chn,P_L1,NORM);
    }
    else
    {
        read_correlator_totals(chn,CA_primary,FINE);
    }

    phase_error = calc_carrier_error(chn,CA_primary,&sign,L1,0.001F,FALSE,NINETY_DEG); 
    // "phase_error" in cycles

    carrier_third_order(chn,phase_error*360.0F,loop_gain[chn], p_myIRQ);


    if(task_counter[chn] < CHANGE_SPACING_06)
    {
        denom =   I_A[CA_primary][chn]*I_A[CA_primary][chn]
                + Q_A[CA_primary][chn]*Q_A[CA_primary][chn]
                + I_A[P_L1][chn]*I_A[P_L1][chn]
                + Q_A[P_L1][chn]*Q_A[P_L1][chn];

                // I^2 + Q^2 (punctual)
        code_lock_thres[CA_primary][chn] +=   I_B[CA_primary][chn]*I_B[CA_primary][chn]
                                            + Q_B[CA_primary][chn]*Q_B[CA_primary][chn];

    }
    else
    {
        // I^2 + Q^2 (punctual)
        denom =   I_B[CA_primary][chn]*I_B[CA_primary][chn]
                + Q_B[CA_primary][chn]*Q_B[CA_primary][chn];

        code_lock_thres[CA_primary][chn] += denom;
    }

    if(denom != 0)
    {
    

        if(task_counter[chn] < CHANGE_SPACING_06)
        {
            phase_offset  = discriminator_gain[spacing[chn]]
                            * (float)(I_A[CA_primary][chn] * I_A[CA_primary][chn]
                                   +  Q_A[CA_primary][chn] * Q_A[CA_primary][chn]
                                   -  I_A[P_L1][chn] * I_A[P_L1][chn]
                                   -  Q_A[P_L1][chn] * Q_A[P_L1][chn]);
        }
        else
        {
            if(task_counter[chn] < CHANGE_CORRELATORS_EL)
            {
                phase_offset  = 8.0F * (float)(I_B[CA_primary][chn] * (I_A[CA_primary][chn] - I_A[P_L1][chn])
                                            +  Q_B[CA_primary][chn] * (Q_A[CA_primary][chn] - Q_A[P_L1][chn]));
            }
            else
            {
                // Change in gain as the hardware generates (E-L)/2
                // Should be scaled by 16 however, the read back of the I_A (E-L)   
                // correlator is in fine mide which needs scaling by 1/16. Therefore 
                // the combination of the two scale factors results in a unity SF. 

                phase_offset  = (float)(I_B[CA_primary][chn] * I_A[CA_primary][chn]
                                     +  Q_B[CA_primary][chn] * Q_A[CA_primary][chn]);
            }
        }

        // Normalize
        phase_offset /= (float)denom;
        
        if(phase_offset > 8.0F)
            phase_offset = 8.0F;

        if(phase_offset < -8.0F)
            phase_offset = -8.0F;

    }
    else
        phase_offset = 0.0F;
                     
    code_second_order(chn,code_loop_gain[chn]*phase_offset/16.0F, p_myIRQ);

    // After the carrier filter has been operating for 400ms start to reduce 
    // the gain by a factor of 0.95 every 10ms.                              


    task_counter[chn]++;

    if( (loop_gain[chn] != 1.0) && (task_counter[chn] > 400) && ((task_counter[chn]%10) == 0) )
    {
        loop_gain[chn] *= 0.95F;
        if(loop_gain[chn] < 1.0F)
            loop_gain[chn] = 1.0F;
    }


    if( (code_loop_gain[chn] != 1.0) && (task_counter[chn] > 30) && ((task_counter[chn]%10) == 0) )
    {
        code_loop_gain[chn] *= 0.95F;
        if(code_loop_gain[chn] < 1.0F)
            code_loop_gain[chn] = 1.0F;
    }

    switch(task_counter[chn])
    {
        // The chip spacing is reduced incrementally. A time delay occurs  
        // between each increment to allow the tracking loop to settle.    
        // In order for reliable tracking the code phase error must be less
        // than the correlation spacing when the chip spacing is changed.  
        // If not the disciminator will be operating in a non-linear region
        // and the acquisition performance will be compromised.            

        case CHANGE_SPACING_06:
			// PQ 090710 the seperation btw E and L is 0.49104 Chip
			
        	// cio << "CHANGE_SPACING_06 1000ms" << endl;
            // Change the chip spacing to approximately 0.6 chips between  
            // early and late for GPS, 0.3 for GLONASS.                    

            // Calculate the value in here empirically. i.e. log the correlators 
            // and calculate the code phase error.  This will give the step      
            // caused by the change in spacing...                                

            Shift_Code_NCO_Phase (chn,CACODE, -10*4,ASYNC,FALSE, p_myIRQ);
            CA_Lag_Mode[chn] = NARROW_CA+N_EARLY+N_12T_SPACING; // 0.6 chip GPS/ 0.3 chip GLONASS
            Set_Lag_Spacing(chn,PRIMARY,CA_Lag_Mode[chn], p_myIRQ);
            Set_Lag_Spacing(chn,SECONDARY,CA_Lag_Mode[chn], p_myIRQ);

            if(sys[chn] == GPS || sys[chn] == INMARSAT)  // JC 30/10/97 INM
                spacing[chn] = POINT_SIX;
            else
                spacing[chn] = POINT_THREE;
            break;

        case CHANGE_SPACING_04:
        	// cio << "CHANGE_SPACING_04 1200ms" << endl;
            // Change the chip spacing to approximately 0.4 chips between
            // early and late for GPS, 0.2 for GLONASS.                 

            Shift_Code_NCO_Phase (chn,CACODE,-1*4,ASYNC,FALSE, p_myIRQ); // Was -2*4, JC 16/11/95
            CA_Lag_Mode[chn] = NARROW_CA+N_EARLY+N_8T_SPACING; // 0.4 chip GPS/ 0.2 chip GLONASS
            Set_Lag_Spacing(chn,PRIMARY,CA_Lag_Mode[chn], p_myIRQ);
            Set_Lag_Spacing(chn,SECONDARY,CA_Lag_Mode[chn], p_myIRQ);
            if(sys[chn] == GPS  || sys[chn] == INMARSAT)  // JC 30/10/97 INM
                spacing[chn] = POINT_FOUR;
            else
                spacing[chn] = POINT_TWO;
            break;

        case CHANGE_SPACING_02:
        	// cio << "CHANGE_SPACING_02 1400ms" << endl;
            // Change the chip spacing to approximately 0.2 chips between 
            // early and late for GPS, GLONASS remains unchanged.         

            if(sys[chn] == GPS || sys[chn] == INMARSAT)  // JC 30/10/97 INM 
            {
                Shift_Code_NCO_Phase (chn,CACODE,-2*4,ASYNC,FALSE, p_myIRQ);
                spacing[chn] = POINT_TWO;
                CA_Lag_Mode[chn] = NARROW_CA+N_EARLY+N_4T_SPACING; // 0.2 chip 
                Set_Lag_Spacing(chn,PRIMARY,CA_Lag_Mode[chn], p_myIRQ);
                Set_Lag_Spacing(chn,SECONDARY,CA_Lag_Mode[chn], p_myIRQ);
            }
            break;

        case CHANGE_CORRELATORS_EL:
        	// cio << "CHANGE_CORRELATORS_EL 1800ms" << endl;
            // The receiver is integrating over 1ms. The chip spacing is  
            // not reduced beyond 0.2 for GPS or GLONASS until the        
            // integration time has been increased to it's maximum value  
            // this is to allow relatively large linear discriminator     
            // region. This is so that if there is a transient when the   
            // integration it will be quickly corrected.                  

            if(sys[chn] == GPS || sys[chn] == INMARSAT)  // JC 30/10/97 INM
            {
                CA_Lag_Mode[chn] = NARROW_CA+N_E_L+N_4T_SPACING; // 0.2 chip E-L
            }
            else
            {
                CA_Lag_Mode[chn] = NARROW_CA+N_E_L+N_8T_SPACING; // 0.2 chip E-L
            }
            Set_Lag_Spacing(chn,PRIMARY,FULL+CA_Lag_Mode[chn], p_myIRQ);
            // Accumulators 3,4,5 and 6 now relinquished ready for P-code
            break;

        default:
            break;
    }

	// check PLL Code energy every 60ms
    if( (task_counter[chn] != 0) && ((task_counter[chn]%60) == 0))
    {
        if(code_lock_thres[CA_primary][chn] < STATE_3_THRES[sys[chn]])
        {	
			//cio << "PLL Code Fail " << code_lock_thres[CA_primary][chn] << normal << endl; cio << noshowcursor << flush;
            return_to_one(chn, p_myIRQ);
        }
        code_lock_thres[CA_primary][chn] = 0;

		int temp = (int)(task_counter[chn]/60); debug_task_counter_60[temp] = code_lock_thres[CA_primary][chn];
    }


    if((task_counter[chn] > END_TIME_STATE_3) && (tracking_state[chn] != STATE_1))
    {
        if(filtered_carr_error[L1][chn] < PHASE_LOCK_THRES)
        {
            if(sys[chn] != INMARSAT)  // JC 30/10/97 INM
			{
                tracking_state[chn] = STATE_4;
			}
            task_counter[chn] = 0;
            code_lock_thres[CA_primary][chn] = 0;
			cio << "PLL OK " << filtered_carr_error[L1][chn] << "<" << PHASE_LOCK_THRES << " " << code_lock_thres[CA_primary][chn] << "<" << STATE_3_THRES[sys[chn]] << normal << endl; cio << noshowcursor << flush;
        }
		else
		{
			//cio << "PLL Carr Fail " << filtered_carr_error[L1][chn] << normal << endl; cio << noshowcursor << flush;
			return_to_one(chn, p_myIRQ);
		}
    }
}


void track_initialize(int chn,int dop,int satsys,int SV,int GPS_ON_GLONASS_FLAG, Irq *p_myIRQ)
{
    volatile float aid;
    
    tracking_state[chn] = UNUSED;

    // obsolete:
    // DPRAM[CHANNELS_ACTIVE] &= ~((0x1 << chn) & 0xffff);
    // PQ: now disable the bit corresponding to the channel in the DPRAM_CHANNELS_ACTIVE_PQ variable
	DPRAM_CHANNELS_ACTIVE_PQ &= ~((0x1 << chn) & 0xffff);

    if(satsys == GLONASS)
    {
 		//cio << bold << "PQ:090706 this code should not be run agsdfgsdg, warning!\n" << normal << endl;
        /* Reset the channel H/W */
        Reset_Channel(chn,GLONASS, p_myIRQ);
        initialize_channel(chn);
        
        if(GPS_ON_GLONASS_FLAG == FALSE)
        {
		    //cio << bold << "disabled GLONASS, warning dfte6u\n" << normal << endl;
            sys[chn] = GLONASS;
            aid = GLONASS_L1_CA_carr_code_ratio[SV-1];
        }
        else
        {
		    //cio << bold << "disabled GLONASS, warning 56gje67e5\n" << normal << endl;
            SV = 12; /* Calibrate at channel 12 */ 
            sys[chn] = GLONASS;
            aid = GLONASS_L1_CA_carr_code_ratio[SV-1];
        }    

        if(SV < GLONASS_FOLDOVER_CHANNEL_L1)/*GB assuming only L1 operation in this function */
        {                                          
            carrier_top[L1][chn] = GLN_L1_TOP[SV];
            carrier_low[L1][chn] = GLN_L1_WORD[SV] - (long)((float)dop*ONE_HZ_WORD_CARR);
            code_freq_word[CA_primary][chn] = GLONASS_NOMINAL_CODE_WORD_INT + (long)(GLONASS_NOMINAL_CODE_WORD_FLOAT + ONE_HZ_WORD_CODE*(float)dop/aid);
            Set_Flags(chn,PRIMARY, INVQ_L1, p_myIRQ);
            Set_Flags(chn,SECONDARY, INVQ_L2, p_myIRQ);
        }
        else
        {
            carrier_top[L1][chn] = GLN_L1_TOP[SV];
            carrier_low[L1][chn] = GLN_L1_WORD[SV] + (long)((float)dop*ONE_HZ_WORD_CARR);
            code_freq_word[CA_primary][chn] = GLONASS_NOMINAL_CODE_WORD_INT + (long)(GLONASS_NOMINAL_CODE_WORD_FLOAT + ONE_HZ_WORD_CODE*(float)dop/aid);
        }

        if(GPS_ON_GLONASS_FLAG == TRUE)
        {
            code_freq_word[CA_primary][chn]*=2;
            Initialise_CA_Code (chn,PRIMARY,GPS,32, p_myIRQ);
            SV = 32;  /* USE PRN 32 */
        }
        else
        {
		    //cio << bold << "disabled GLONASS, warning 56fhjf6u\n" << normal << endl;
            Initialise_CA_Code (chn,PRIMARY,GLONASS,SV, p_myIRQ);
        }    
        
    }        
    else if(satsys == GPS)
    {
        /* Reset the channel H/W */
        Reset_Channel(chn,GPS, p_myIRQ);
        initialize_channel(chn);

        sys[chn] = GPS; 

        carrier_top[L1][chn] = GPS_L1_TOP;
        carrier_low[L1][chn] = GPS_L1_WORD + (long)((float)dop*ONE_HZ_WORD_CARR);
        
        /* Pre-calculated value of the code freq offset by predicted doppler */
        code_freq_word[CA_primary][chn] = GPS_NOMINAL_CODE_WORD_INT + (long)(GPS_NOMINAL_CODE_WORD_FLOAT + ONE_HZ_WORD_CODE*(float)dop/1540.0F);
        
        /* FORCE NO TRACKING INTERFERENCE WORK */
/*
        carrier_top[L1][chn] = GPS_L1_TOP;
        carrier_low[L1][chn] = GPS_L1_WORD;
        code_freq_word[CA_primary][chn] = GPS_NOMINAL_CODE_WORD_INT + (long)(GPS_NOMINAL_CODE_WORD_FLOAT);
*/
        /* END */

        Initialise_CA_Code (chn,PRIMARY,GPS,SV, p_myIRQ);
    }
    else   /* JC 30/10/97 INM */
    {
        /* Reset the channel H/W */
        Reset_Channel(chn,GPS, p_myIRQ);
        initialize_channel(chn);

        sys[chn] = INMARSAT; 

        carrier_top[L1][chn] = GPS_L1_TOP;
        carrier_low[L1][chn] = GPS_L1_WORD + (long)((float)dop*ONE_HZ_WORD_CARR);
        
        /* Pre-calculated value of the code freq offset by predicted doppler */
        code_freq_word[CA_primary][chn] = GPS_NOMINAL_CODE_WORD_INT + (long)(GPS_NOMINAL_CODE_WORD_FLOAT + ONE_HZ_WORD_CODE*(float)dop/1540.0F);
        
        Initialise_CA_Code (chn,PRIMARY,INMARSAT,SV,p_myIRQ);
    }


    /* The a priori frequency estimate is calculated by using the PC supplied estimate and the */
    /* IF frequency word from look-up-tables. The frequency word is partitioned into a 24 and  */
    /* a 12 bit word. To prevent incorrect wrap over the sign and magnitude must be checked    */
    /* and appropriate action taken.                                                           */
    
    carrier_word_wrap_loop(chn,L1);
        
    channel_num[chn] = SV;
    
    /* Store inital value of NCO so that if a false acquisition */
    /* occurs the carr H/W can be reset to the inital value */
    carrier_low_start[chn] = carrier_low[L1][chn];
    carrier_top_start[chn] = carrier_top[L1][chn];

    /* Used for re-initialization of the code NCO */
    start_code_freq[chn] = code_freq_word[CA_primary][chn];

    Set_Code_NCO(chn,CACODE,code_freq_word[CA_primary][chn],p_myIRQ);
	// PQ 20080327 found NCO was NOT started as the new FCW is not updated due to no epoch signal.

    DISABLEINTERRUPTS;
    NOPDELAY();
    *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_NCO3FCW_ACT)) = 0;
    NOPDELAY();
	ENABLEINTERRUPTS;

    Set_Carrier_NCO (chn,L1CARRIER,carrier_top[L1][chn],carrier_low[L1][chn], p_myIRQ);
    
    Stop_Integration(chn,PRIMARY,p_myIRQ); // PQ added in order to ensure INTEG = 0
    if(GPS_ON_GLONASS_FLAG == TRUE)
        Set_Dwell_Timing(chn,PRIMARY,ONE_MS,GPS,p_myIRQ);
    else    
        Set_Dwell_Timing(chn,PRIMARY,ONE_MS,sys[chn],p_myIRQ);


    CA_Lag_Mode[chn] = WIDE_CA+W_EARLY+W_2_SPACING; /* 2 chips */
    Set_Lag_Spacing(chn,PRIMARY,CA_Lag_Mode[chn],p_myIRQ);
    Set_Lag_Spacing(chn,SECONDARY,CA_Lag_Mode[chn],p_myIRQ);
    Start_Integration(chn,PRIMARY,p_myIRQ);
    tracking_state[chn] = STATE_0;
//	unsigned int* RW_CTRL1 = (unsigned int*)0x80210418;
//	unsigned int temp = *RW_CTRL1; // Debug purpose, it reads back 010, i.e.,INTEGRATE1 = 1
}



void Generate_P_Code_Tables(void)
{ 
	//////////////////////////////////
	// PQ: function to be checked
	//////////////////////////////////
  int Chip, X2A_State, X2B_State;

  /* calculate states for x2a and x2b... */
  X2A_State=2341; /* initial state of x2a */
  X2B_State=1364; /* initial state of x2b */
  for (Chip=0; Chip<=4092; Chip++)
    { 
    	X2_Codes[Chip] = (X2A_State) + (X2B_State << 12);
     	X2A_State = ((X2A_State<<1) & 4095) + (1 & (
        	X2A_State^(X2A_State>>2)^(X2A_State>>3)^(X2A_State>>4)^(X2A_State>>6)
        	^(X2A_State>>7)^(X2A_State>>8)^(X2A_State>>9)^(X2A_State>>10)^(X2A_State>>11) ));
     	X2B_State = ((X2B_State<<1) & 4095) + (1 & (
        	(X2B_State>>1)^(X2B_State>>2)^(X2B_State>>3)
        	^(X2B_State>>7)^(X2B_State>>8)^(X2B_State>>11) ));
    }
}


void Reset_Channel (int chn, int Constellation, Irq *p_myIRQ)
{
    int sys;

    // channel initialised for single CA-code search

    Write_Flags(chn,GLOBAL,L1_on_L2, p_myIRQ);
    // L1 counter-rotated samples are on L2
    // so that those accumulators can be used
    // during searching.
    // The `Dual C/A', `Zero_L1' and `Zero_L2' flags are cleared.
    
    Write_Flags(chn,P1CTRL,CORRELATE, p_myIRQ);
    Write_Flags(chn,P2CTRL,CORRELATE, p_myIRQ);
    // Enable correlation with code for correlators 3 to 6
    // The `Two-stage accumulator' flags are cleared.
    // In P1CTRL, the `L2ONACC3' and `L2ONACC4' flags are cleared.
    
    sys = (Constellation == GPS || Constellation == INMARSAT) ? GPS_ADC : GLONASS_ADC;
    Write_Flags(chn,PRIMARY,L1_BAND | sys, p_myIRQ); // use 
    Write_Flags(chn,SECONDARY,L2_BAND | sys, p_myIRQ);

    /* The `INV_Q', `INTEGRATE', `TIMARM' flags are clear.
       The `INV_I flag in SECONDARY is clear.
       L1 samples are fed through the L1 carrier NCO;
       L2 samples are fed through the L2 carrier NCO.
    */

    /* Cascade code laggers, ready for search */
    Set_Lag_Spacing(chn,PRIMARY,   WIDE_CA|W_EARLY|W_2_SPACING, p_myIRQ);
    Set_Lag_Spacing(chn,SECONDARY, WIDE_CA|W_EARLY|W_2_SPACING, p_myIRQ);

    /* Halt all NCOs (on next epoch). */
    
    Set_Carrier_NCO(chn,L1CARRIER,0,0, p_myIRQ);
    Set_Carrier_NCO(chn,L2CARRIER,0,0, p_myIRQ);
    Set_Code_NCO(chn,CACODE,0, p_myIRQ);
    Set_Code_NCO(chn,P1CODE,0, p_myIRQ);
    Set_Code_NCO(chn,P2CODE,0, p_myIRQ);
    Set_Cycle_Difference(chn,0, p_myIRQ);


    Stop_Integration(chn,PRIMARY, p_myIRQ);
    Start_Integration(chn,PRIMARY, p_myIRQ);
    // PQ 090706 initialize the channel to be GPS
    //Initialise_CA_Code(chn,PRIMARY,GLONASS,0, p_myIRQ);
    Initialise_CA_Code(chn,PRIMARY,GPS,0, p_myIRQ);
    Stop_Integration(chn,PRIMARY, p_myIRQ);
    
    Shift_Code_NCO_Phase(chn,CACODE,1,ASYNC,TRUE, p_myIRQ);
    Shift_Code_NCO_Phase(chn,P1CODE,1,ASYNC,TRUE, p_myIRQ);
    Shift_Code_NCO_Phase(chn,P2CODE,1,ASYNC,TRUE, p_myIRQ);
    Shift_Code_NCO_Phase(chn,CACODE,1,ASYNC,FALSE, p_myIRQ);
    Shift_Code_NCO_Phase(chn,P1CODE,1,ASYNC,FALSE, p_myIRQ);
    Shift_Code_NCO_Phase(chn,P2CODE,1,ASYNC,FALSE, p_myIRQ);

    Set_Flags(chn,GLOBAL,ZERO_L1 | ZERO_L2, p_myIRQ);
    Clear_Flags(chn,GLOBAL,ZERO_L1 | ZERO_L2, p_myIRQ);
}

void Write_Flags (int chn, int Register, int Value, Irq *p_myIRQ)
{

    DISABLEINTERRUPTS;


    switch(Register)
    {
        case GLOBAL :
            *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_FLAGS)) = Value;
            break;

        case PRIMARY :
            *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_CTRL1)) = Value;
            break;

        case SECONDARY :
            *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_CTRL2)) = Value;
            break;

        case P1CTRL :
            *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_CTRL3)) = Value;
            break;

        case P2CTRL :
            *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_CTRL4)) = Value;
            break;
    }
    ENABLEINTERRUPTS;
}


void Set_Lag_Spacing (int chn, int SubChannel, int Mode, Irq *p_myIRQ)
{
    switch(SubChannel)
    {
        case PRIMARY :
            DISABLEINTERRUPTS;

            NOPDELAY();
            *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_EPL1)) = Mode;
            NOPDELAY();

            ENABLEINTERRUPTS;
            break;  /* Don't know if this should be here so added - JC */

        case SECONDARY :
            DISABLEINTERRUPTS;

            NOPDELAY();
            *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_EPL2)) = Mode;
            NOPDELAY();

            ENABLEINTERRUPTS;
            break;
    }
}


void Set_Carrier_NCO (int chn, int NCO, long Fcw_Hi, long Fcw_Lo, Irq *p_myIRQ)
{
    unsigned int Targeted_CARNCO_FCW_addr;

    DISABLEINTERRUPTS;

    switch(NCO)
    {
        case L1CARRIER :
            Targeted_CARNCO_FCW_addr =  WR_CARNCO1;
            break;

        case L2CARRIER :
            Targeted_CARNCO_FCW_addr =  WR_CARNCO2;
            break;
    }

    NOPDELAY();
    *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + Targeted_CARNCO_FCW_addr)) =
        Fcw_Hi & 0x0F;

    NOPDELAY();
    *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + Targeted_CARNCO_FCW_addr)) =
        Fcw_Lo & 0x0FFFFFFFF; // 081022

    NOPDELAY();

    ENABLEINTERRUPTS;
}

void Set_Code_NCO (int chn, int NCO, unsigned long Frequency_Word, Irq *p_myIRQ)
{
    int Targeted_CodeNCO_FCW_addr;

    switch(NCO)
    {
        case CACODE:
            Targeted_CodeNCO_FCW_addr =  WR_NCO3FCW;
            break;

        case P1CODE:
            Targeted_CodeNCO_FCW_addr =  WR_NCO4FCW;
            break;

        case P2CODE:
            Targeted_CodeNCO_FCW_addr =  WR_NCO5FCW;
            break;
    }

    DISABLEINTERRUPTS;

    NOPDELAY();
    *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + Targeted_CodeNCO_FCW_addr)) =
        Frequency_Word & 0x0FFFFFFFF; // 081022

    NOPDELAY();

	ENABLEINTERRUPTS;
}

void Set_Cycle_Difference (int chn, int Delay, Irq *p_myIRQ)
{
    DISABLEINTERRUPTS;

    NOPDELAY();
    *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_DIFF)) = (Delay + 3);
    NOPDELAY();

    ENABLEINTERRUPTS;
}


void Stop_Integration (int chn, int SubChannel, Irq *p_myIRQ)
{
    Clear_Flags(chn, SubChannel, INTEGRATE, p_myIRQ);
}

void Clear_Flags (int chn, int Register, int ClearMask, Irq *p_myIRQ)
{
    DISABLEINTERRUPTS;

    switch(Register)
    {
        case GLOBAL :
            *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_FLAGS)) =
          ((*((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + RD_FLAGS)) & 0xF) & ~ClearMask);
            break;

        case PRIMARY :
            *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_CTRL1)) =
          ((*((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + RD_CTRL1)) & 0x3F) & ~ClearMask);
            break;

        case SECONDARY :
            *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_CTRL2)) =
          ((*((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + RD_CTRL2)) & 0x7F) & ~ClearMask);
            break;

        case P1CTRL :
            *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_CTRL3)) =
          ((*((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + RD_CTRL3)) & 0xF) & ~ClearMask);
            break;

        case P2CTRL :
            *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_CTRL4)) =
          ((*((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + RD_CTRL4)) & 0x3) & ~ClearMask);
            break;
    }

    ENABLEINTERRUPTS;
}

void Start_Integration (int chn, int SubChannel, Irq *p_myIRQ)
{
    Set_Flags(chn, SubChannel, INTEGRATE, p_myIRQ);
}

void Set_Flags (int chn, int Register, int SetMask, Irq *p_myIRQ)
{
    DISABLEINTERRUPTS;

    switch(Register)
    {
        case GLOBAL :
            *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_FLAGS)) =
          ((*((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + RD_FLAGS)) & 0xF) | SetMask);
            break;

        case PRIMARY :
            *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_CTRL1)) =
          ((*((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + RD_CTRL1)) & 0x3F) | SetMask);
            break;

        case SECONDARY :
            *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_CTRL2)) =
          ((*((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + RD_CTRL2)) & 0x7F) | SetMask);
            break;

        case P1CTRL :
            *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_CTRL3)) =
          ((*((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + RD_CTRL3)) & 0xF) | SetMask);
            break;

        case P2CTRL :
            *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_CTRL4)) =
          ((*((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + RD_CTRL4)) & 0x3) | SetMask);
            break;
    }

    ENABLEINTERRUPTS;
}

void Initialise_CA_Code (int chn, int SubChannel, int Constellation, int Satellite, Irq *p_myIRQ)
{
    int Targeted_CA_generator_addr;

    Targeted_CA_generator_addr = (SubChannel == PRIMARY) ?  WR_GEN1 : WR_GEN5;

    DISABLEINTERRUPTS;

    if (Constellation == GLONASS)
	{
	    //cio << bold << "disabled GLONASS, warning 456uyyudyuj\n" << normal << endl;
        *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + Targeted_CA_generator_addr)) = ALL_ONES_GLN;
    }
    else
    {
//        if(Constellation == GPS)
//            *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + Targeted_CA_generator_addr)) = GenInit[Satellite];
//        else    /* JC 30/10/97 INM */
//            *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + Targeted_CA_generator_addr)) = GenInitINM[0];
//
//        *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + Targeted_CA_generator_addr)) = ALL_ONES_GPS;

// G2          *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + Targeted_CA_generator_addr)) = 22;
//		  NOPDELAY();
// G1         *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + Targeted_CA_generator_addr)) = 511;

//		unsigned int tt = 53*1024+1023;
//         *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + Targeted_CA_generator_addr)) = tt;
		
		unsigned int temp;
        if(Constellation == GPS)
		{
			temp = GenInit[Satellite]*1024 + 1023; // G2 is bit 19->10, G1 is bit 9->0
			// Step to set bit 20 to be 1 to specify GPS, instead of Glonass, is 
			// included in the value of GenInit[Satellite]
		}
        else
		{
			temp = GenInitINM[0]*1024 + 1023;
		}
        *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + Targeted_CA_generator_addr)) = temp;
    }
    ENABLEINTERRUPTS;
 	//unsigned int temp = *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + RD_GEN1));//  DEBUG purpose
	//NOPDELAY(); // to be removed. Debug only.

}

void Shift_Code_NCO_Phase (int chn, int NCO, int Sixtyfourths,int SyncMode, int Zero, Irq *p_myIRQ)
{ 
    int Code_NCO_PHZ_Shift_CW;

    if(Sixtyfourths!=0)
    {
        if(SyncMode == ASYNC)
            Code_NCO_PHZ_Shift_CW = 2048;
    	else
            Code_NCO_PHZ_Shift_CW = 0;

        if(Zero)
            Code_NCO_PHZ_Shift_CW += 512;

        if(Sixtyfourths < 0)
            Code_NCO_PHZ_Shift_CW += (1024 - Sixtyfourths);
    	else
            Code_NCO_PHZ_Shift_CW += Sixtyfourths;

	    switch(NCO)
	    {
	  		case CACODE :
    			DISABLEINTERRUPTS;
                NOPDELAY();
                *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_NCO3PHZ)) = Code_NCO_PHZ_Shift_CW;
    			NOPDELAY();
                ENABLEINTERRUPTS;
                break;

            case P1CODE :
    			DISABLEINTERRUPTS;
                NOPDELAY();
                *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_NCO4PHZ)) = Code_NCO_PHZ_Shift_CW;
    			NOPDELAY();
                ENABLEINTERRUPTS;
                break;

            case P2CODE :
    			DISABLEINTERRUPTS;
                NOPDELAY();
                *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_NCO5PHZ)) = Code_NCO_PHZ_Shift_CW;
    			NOPDELAY();
                ENABLEINTERRUPTS;
                break;
    	}
    }
}

void Read_Accumulators(int chn, int Bank, int *AccI, int *AccQ)
{
	int temp_AccI, temp_AccQ;
    switch(Bank)
    {
        case EARLYCA :

            temp_AccI = *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + RD_ACC1I));
            temp_AccQ = *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + RD_ACC1Q));
            break;

        case PUNCTCA :
            temp_AccI = *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + RD_ACC2I));
            temp_AccQ = *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + RD_ACC2Q));
            break;

        case EARLYP1 :
            temp_AccI = *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + RD_ACC3I));
            temp_AccQ = *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + RD_ACC3Q));
            break;

        case PUNCTP1 :
            temp_AccI = *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + RD_ACC4I));
            temp_AccQ = *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + RD_ACC4Q));
            break;

        case EARLYP2 :
            temp_AccI = *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + RD_ACC5I));
            temp_AccQ = *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + RD_ACC5Q));
            break;

        case PUNCTP2 :
            temp_AccI = *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + RD_ACC6I));
            temp_AccQ = *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + RD_ACC6Q));
            break;

        default :
            temp_AccI = 0;
            temp_AccQ = 0;
            break;
    }
	temp_AccI = temp_AccI & 0x7FFFFF;
	temp_AccQ = temp_AccQ & 0x7FFFFF;

    if((temp_AccI & 0x400000) == 0)
        *AccI = temp_AccI;
    else
        *AccI = (temp_AccI - 0x800000);

    if((temp_AccQ & 0x400000) == 0)
        *AccQ = temp_AccQ;
    else
        *AccQ = (temp_AccQ - 0x800000);
}

void Set_Dwell_Timing(int chn, int SubChannel, int Mode, int Constellation, Irq *p_myIRQ)
/*
  Inputs:   Subchannel designator (PRIMARY or SECONDARY),
        Dwell time designator (QUARTER_MS, HALF_MS or number of 
          C/A code epochs between 1 and 31),
        Constellation designator (GPS or GLONASS).
  Outputs:  hardware
  Calls:    Change_Dwell_Timing
  Function: Sets the dwell time for the accumulators before starting
        integrations.
  Author:   Neil Howard
*/
/*
   Sets dwell timing. Must be done when the channel
   is not integrating already. Dwells will start
   synchronised with code epoch.

   Mode = 1 : 1/4 chip dwell time.
   Mode = 2 : 1/2 chip dwell time.
   Mode = 3 : 1   chip dwell time.
   Mode = 4 : 1   bit dwell time.
*/
{
    Change_Dwell_Timing(chn, SubChannel, Mode, Constellation, p_myIRQ);
    Change_Dwell_Timing(chn, SubChannel, Mode, Constellation, p_myIRQ); // reason to write twice, refer to ESA print-out
}

void Change_Dwell_Timing (int chn, int SubChannel, int Mode, int Constellation, Irq *p_myIRQ)
/*
  Inputs:   Subchannel designator (PRIMARY or SECONDARY),
        Dwell time designator (QUARTER_MS, HALF_MS or number of 
          C/A code epochs between 1 and 31),
        Constellation designator (GPS or GLONASS).
  Outputs:  hardware
  Calls:    none
  Function: Changes the dwell time for the accumulators, taking effect at the
        end of the current dwell period.
  Author:   Neil Howard
*/
/*
   Changes dwell timing on next code epoch.
   To be used when already integrating.

   Constellation : GLONASS or GPS.
   Mode = 1 : 1/4 chip dwell time.
   Mode = 2 : 1/2 chip dwell time.
   Mode = 3 : 1   chip dwell time.
   Mode = 4 : 1   bit dwell time.

   Change between modes will take effect at end of
   current integration period. Care should be taken
   changing modes. A change from mode 1 to mode 2 should
   be done on the 2nd or 4th quarters of the chip. A
   change from mode 3 to 4 should be done during last chip
   of the bit. Otherwise, a number of incorrect integration
   periods will result, before the Integration timer settles
   down producing the correct periods. In some cases it may
   be preferable to stop integrating, set up the new dwell time
   (with `Set_Dwell_Timing') and restart integration (which
   will take effect on the next code epoch), even though this
   will result in loss of integration data for a short period.
*/
{
    int Temp;

    switch(Constellation)
    {
        case GLONASS :
            switch(Mode)
            {
                case QUARTER_MS :
                    Temp = 127;
                    break;
                case HALF_MS :
                    Temp = 255;
                    break;
                default :
                    Temp = Mode-1;
                    break;
            }
            break;

        case GPS :
        case INMARSAT :  /* JC 30/10/97 INM */
            switch(Mode)
            {
                case QUARTER_MS :
                    Temp = 255;
                    break;
                case HALF_MS :
                    Temp = 511;
                    break;
                default :
                    Temp = Mode-1;
                    break;
            }
            break;
    }

    switch(SubChannel)
    {
        case PRIMARY :
            DISABLEINTERRUPTS;

            NOPDELAY();
            *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_INT1)) = Temp;
           	NOPDELAY();

            ENABLEINTERRUPTS;
            break;

        case SECONDARY :
            DISABLEINTERRUPTS;

            NOPDELAY();
            *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_INT2)) = Temp;
            NOPDELAY();

            ENABLEINTERRUPTS;
            break;

    }
}

void Initialise_P_Code(int chn, int Constellation, int Satellite, int SubFrame, Irq *p_myIRQ)
/*
  Inputs:   Constellation designator (GPS or GLONASS),
        Satellite identifer (for GPS),
        Hand-over number (for GPS)
  Outputs:  hardware
  Calls:    none
  Function: Sets and initialises the P code generator for a 
        particular satellite at a particular time of the week.
  Author:   Neil Howard
*/
/*
  Constellation: `GPS' or `GLONASS'.
  Satellite:     SV number for GPS; ignored by GLONASS.
  Subframe:      For GPS, 17 bit no.= Z-count*4 
         (No. of 6-second subframes into code sequence).
         For GLONASS, ignored.
*/
{ 
    int Z_Count, X2_Chip, Status_Reg;
    int X2A_Chip, X2A_State, X2A_Frozen;
    int X2B_Chip, X2B_State, X2B_Halted;
 
    if (Constellation==GPS)
    {
        Z_Count = (4*SubFrame); /* no. of X1 cycles into sequence */

        /* X1 Always initialised to the start of the sequence...
           X1A Code = 584
           X1B Code = 1364
        */

        /* Precession of X1 relative to X2 of (37*4) chips per subframe
           and by constant satellite number
        */
        X2_Chip = 15345037-(37*Z_Count)-Satellite;

        /* Calculate where to initialise X2A and X2B */
        if (X2_Chip >= 15345000)
        {
            /* X2A is frozen */
            X2A_Chip=(X2_Chip-15345000);
            X2A_Frozen = 1;
        }
        else
        {
            X2A_Chip = (X2_Chip%4092);
            X2A_Frozen = 0;
        }
        if (X2_Chip >= 15344657)
        {
            /* X2B is halted in final state */
            X2B_Chip = 4092;
            X2B_Halted = 1;
        }
        else
        {
            X2B_Chip = (X2_Chip % 4093);
            X2B_Halted = 0;
        }
        Status_Reg = (GPS<<2) + (X2A_Frozen<<1) + (X2B_Halted);

        /* calculate states for x2a and x2b... */
        X2A_State = (X2_Codes[X2A_Chip] & 4095);
        X2B_State = ((X2_Codes[X2B_Chip] >> 12) & 4095);

        DISABLEINTERRUPTS;
        NOPDELAY();
        *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_GEN2_SETUP)) = Status_Reg;
        NOPDELAY();
        *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_GEN2_x2a)) = X2A_State;
        NOPDELAY();
        *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_GEN2_x2b)) = X2B_State;
        NOPDELAY();
        *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_GEN2_SETUP)) = Status_Reg;
        ENABLEINTERRUPTS;
    }
    /* This else is also run with INMARSAT although it doesn't matter! */
    else /* Constellation = GLONASS */
    {
        Status_Reg = (GLONASS<<2);

        /* Set up code generator for GLONASS */
        DISABLEINTERRUPTS;
        NOPDELAY();
        *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_GEN2_SETUP)) = Status_Reg;
        NOPDELAY();
        *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_GEN2_x2a)) = 0;
        NOPDELAY();
        *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_GEN2_x2b)) = 0;
        NOPDELAY();
        *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_GEN2_SETUP)) = Status_Reg;

        ENABLEINTERRUPTS;
    }
}

void Initialise_W_Control (int chn, int A, int B, int M, int W1_Estimates, int W2_Estimates, int Acc_Control, Irq *p_myIRQ)
/*
  Inputs:   W-code edge generation parameters (A, B and M),
        source designators of the W-code decryption signals for L1 and L2,
        Designator for the control of accumulations over the A and B phases 
        of the W-code edge generator.
  Outputs:  hardware
  Calls:    none
  Function: Sets the W-code edge generation parameters (A, B and M). 
        Enables/disables accumulations over the A and B phases of the W-code
        edge generator.
        Sets the sources of the W-code decryption signals for L1 and L2.
  Author:   Neil Howard
*/
/*
     A, B, M : parameters for W edge generation
     W1_Estimates, W2_Estimates:
       one of:
     NO_W         Always add primary accumulators to secondary ones.
     CROSS_W      e.g. use L2 W estimates to remove W code from L1.
     SQUARE_W     e.g. use L1 W estimates to remove W code from L1.
*/
{
    int Temp1, Temp2;

    Temp1 = (M & 0xFF)
            + ((W1_Estimates & 0x3)<<8)
            + ((W2_Estimates & 0x3)<<10);
    Temp2 = (A & 0x1F) + ((B & 0x1F)<<5) + Acc_Control;

    DISABLEINTERRUPTS;

    NOPDELAY();
    *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_GEN4_m)) = Temp1;
    NOPDELAY();
    *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_GEN4_ab)) = Temp2;
    NOPDELAY();
    *((unsigned int *)(CH_HARDWARE_BASE + chn * HARDWARE_OFFSET + WR_GEN4_m)) = Temp1;
    NOPDELAY();

    ENABLEINTERRUPTS;
}


void Read_Interrupts(void)
{
    unsigned int ReadIntCount = 0x0000; // index of current active channel stored in array active_channels
    unsigned int Read; // read back value from the hardware registers.

    /* Enable and disable interrupts are removed because this is inside ISR */

    tasks.NumberOneMs = 0;  /* Reset previous count */
    tasks.NumberTwoMs = 0;  /* Reset previous count */
	occur_Int ++;
    while((ReadIntCount < NO_CHAN) && (active_channels[ReadIntCount] != ACTIVE_CHANNELS_END))
    {
/*        // Read the interrupt flag from hardware
		//Read = *((unsigned int *)0x80210210);
        //Read = *((unsigned int *)0x80210274);// + active_channels[ReadIntCount] * HARDWARE_OFFSET + RD_DEBUG2);
        Read = *((unsigned int *)(CH_HARDWARE_BASE + RD_DEBUG1));
        Read = *((unsigned int *)(CH_HARDWARE_BASE + RD_DEBUG2));
        Read = *((unsigned int *)(CH_HARDWARE_BASE + RD_DEBUG3));
        Read = *((unsigned int *)(CH_HARDWARE_BASE + RD_DEBUG4));
        Read = *((unsigned int *)(CH_HARDWARE_BASE + RD_DEBUG5));

        Read = *((unsigned int *)(CH_HARDWARE_BASE + active_channels[ReadIntCount] * HARDWARE_OFFSET + RD_DEBUG1));
        Read = *((unsigned int *)(CH_HARDWARE_BASE + active_channels[ReadIntCount] * HARDWARE_OFFSET + RD_DEBUG2));
        Read = *((unsigned int *)(CH_HARDWARE_BASE + active_channels[ReadIntCount] * HARDWARE_OFFSET + RD_DEBUG3));
        Read = *((unsigned int *)(CH_HARDWARE_BASE + active_channels[ReadIntCount] * HARDWARE_OFFSET + RD_DEBUG4));
        Read = *((unsigned int *)(CH_HARDWARE_BASE + active_channels[ReadIntCount] * HARDWARE_OFFSET + RD_DEBUG5));*/
        Read = *((unsigned int *)(CH_HARDWARE_BASE + active_channels[ReadIntCount] * HARDWARE_OFFSET + RD_FLAGS));
        if((Read & INTPRIM)!=0)
        {
			occur_Int_with401 ++;
            // keep other bits unchanged and clear interrupt 
			*((unsigned int *)(CH_HARDWARE_BASE + active_channels[ReadIntCount] * HARDWARE_OFFSET +  WR_FLAGS)) =
               	((
               		*((unsigned int *)(CH_HARDWARE_BASE + active_channels[ReadIntCount] * HARDWARE_OFFSET + RD_FLAGS))
				& 0xF) | INTPRIM);
				
            // Load this interrupt into the task structure to be processed
            load_task_structure(active_channels[ReadIntCount]);

			// PQ: 080117 next few lines were outside the bracket.
			// It is because the interrupt could happen when there is no channel IRQ, 
			// and the previous stored channel_state_log would be used and it is not correct.

	        // Count the number of tasks operating at 1ms for the scheduler
	        if(channel_state_log[active_channels[ReadIntCount]] == 1)
	            tasks.NumberOneMs++;
	        if(channel_state_log[active_channels[ReadIntCount]] == 2)
	            tasks.NumberTwoMs++;
        }
        ReadIntCount++;
    }
}


void read_correlator_totals(int chn,int sub_chn,int res)
{
    switch(sub_chn)
    {
        case CA_primary:

            /* Early */
            Read_Accumulators(chn,EARLYCA,&I_A[sub_chn][chn],&Q_A[sub_chn][chn]);

            if(res == NORM)
            {
                I_A[sub_chn][chn]/=16;
                Q_A[sub_chn][chn]/=16;
            }

            /* Punctual */
            Read_Accumulators(chn,PUNCTCA,&I_B[sub_chn][chn],&Q_B[sub_chn][chn]);
            I_B[sub_chn][chn]/=16;
            Q_B[sub_chn][chn]/=16;
            break;
            
        case CA_secondary:
            break;
            
        case P_L1:

            /* Late / Early P */
            /****************************************************************/
            /*  NOTE:- There is only a single L1 carrier NCO which is       */
            /*  controlled by the C/A code tracking loop. The P-code is 90° */
            /*  out of phase from the C/A code and therefore when the       */
            /*  tracking changes to P-code acquisition/tracking the I & Q   */
            /*  correlators must be swapped to maintain compatability with  */
            /*  the tracking software.                                      */
            /****************************************************************/
            if(tracking_state[chn] >= STATE_7)
			{
                Read_Accumulators(chn,EARLYP1,&Q_A[sub_chn][chn],&I_A[sub_chn][chn]);
			}
            else
			{
                Read_Accumulators(chn,EARLYP1,&I_A[sub_chn][chn],&Q_A[sub_chn][chn]);
			}
            if(res == NORM)
            {
                I_A[sub_chn][chn]/=16;
                Q_A[sub_chn][chn]/=16;
            }

            /* V Late / Punctual P */
            if(tracking_state[chn] >= STATE_7)
			{
                Read_Accumulators(chn,PUNCTP1,&Q_B[sub_chn][chn],&I_B[sub_chn][chn]);
			}
            else
			{
                Read_Accumulators(chn,PUNCTP1,&I_B[sub_chn][chn],&Q_B[sub_chn][chn]);
			}
                
            I_B[sub_chn][chn]/=16;
            Q_B[sub_chn][chn]/=16;

            break;
            
        case P_L2:

            /* Late / Early P */

            Read_Accumulators(chn,EARLYP2,&I_A[sub_chn][chn],&Q_A[sub_chn][chn]);

            if(res == NORM)
            {
                I_A[sub_chn][chn]/=16;
                Q_A[sub_chn][chn]/=16;
            }

            /* V Late / Punctual P */
            Read_Accumulators(chn,PUNCTP2,&I_B[sub_chn][chn],&Q_B[sub_chn][chn]);

            I_B[sub_chn][chn]/=16;
            Q_B[sub_chn][chn]/=16;

            break;
            
        default:
            error_type = INVALID_SUB_CHANNEL;
            break;
    }
}

void carrier_word_wrap(int chn,int band,int mod_offset)
{
    /* Account for the carrier word being in two parts */
    if(carrier_low[band][chn] < 0L)
    {   
        if(mod_offset == TRUE)
            freq_offset_top[chn]--;
            
        carrier_top[band][chn]--;
        carrier_low[band][chn] += TWO_POWER_32;
    }
    else
    {
        if(carrier_low[band][chn] >= TWO_POWER_32)
        {
            if(mod_offset == TRUE)
                freq_offset_top[chn]++;
                
            carrier_top[band][chn]++;
            carrier_low[band][chn] -= TWO_POWER_32;
        }
    }
}


void carrier_word_wrap_loop(int chn,int band)
{
    if(carrier_low[band][chn] < 0L)
    {
        /* check for negative word & correct */
        while(carrier_low[band][chn] < 0L)
        {
            carrier_low[band][chn] += TWO_POWER_32;
            carrier_top[band][chn]--;
        }
    }
    else
    {
        /* check for an out of range word and correct */
        while(carrier_low[band][chn] > TWO_POWER_32)
        {
            carrier_low[band][chn] -= TWO_POWER_32;
            carrier_top[band][chn]++;
        }
    }
}



// PQ moved from sched.cpp
void reset_active_channels_matrix(void)
{
    int i;

    for(i=0; i<NO_CHAN; i++)
	{
        active_channels[i] = i;
	}

    active_channels[NO_CHAN] = ACTIVE_CHANNELS_END;
}

void initialize_scheduler(void)
{
    int i;
    
    for(i=0; i<(NO_CHAN + 1); i++)
    {
        tasks.one_ms_tasks[i] = 0;
        tasks.two_ms_tasks[i] = 0;
        tasks.five_ms_tasks[i] = 0;
        tasks.ten_ms_tasks[i] = 0;
    }
    for(i=0; i<NO_CHAN; i++)
    {
        Pending[i].Flag    = FALSE;
        Pending[i].Doppler = 0;
        Pending[i].System  = 0;
        Pending[i].SV      = 0;
        Pending[i].GPSONGLNFlag = FALSE;

        channel_state_log[i] = 0;
    }
    
    tasks.NumberOneMs  = 0;
    tasks.NumberTwoMs  = 0;
    tasks.NumberFiveMs = 0;
    tasks.NumberTenMs  = 0;

    tasks.top_one_ms_buffer = 0;
    tasks.bot_one_ms_buffer = 0;
    
    tasks.top_two_ms_buffer = 0;
    tasks.bot_two_ms_buffer = 0;
    
    tasks.top_five_ms_buffer = 0;
    tasks.bot_five_ms_buffer = 0;
    
    tasks.top_ten_ms_buffer = 0;
    tasks.bot_ten_ms_buffer = 0;
}


void Scheduler(Irq *p_myIRQ)
{
    ProcessOneMs(p_myIRQ);
    ProcessTwoMs(p_myIRQ);
    ProcessFiveMs(p_myIRQ);
    ProcessTenMs(p_myIRQ);
    Process1pps(0);

    if((tasks.NumberOneMs < 2) && (tasks.NumberTwoMs == 0))
	{
        AcquireChannel(p_myIRQ);
	}
}


void ProcessOneMs(Irq *p_myIRQ)
{

    while(tasks.top_one_ms_buffer != tasks.bot_one_ms_buffer)
    {
        track(tasks.one_ms_tasks[tasks.bot_one_ms_buffer], p_myIRQ);
        
        tasks.bot_one_ms_buffer++;
        
        tasks.bot_one_ms_buffer %= (NO_CHAN + 1);
    }
}


void ProcessTwoMs(Irq *p_myIRQ)
{   

    
    while(tasks.top_two_ms_buffer != tasks.bot_two_ms_buffer)
    {
        ProcessOneMs(p_myIRQ);
                 
        track(tasks.two_ms_tasks[tasks.bot_two_ms_buffer], p_myIRQ);
        
        tasks.bot_two_ms_buffer++;
        
        tasks.bot_two_ms_buffer %= (NO_CHAN + 1);
        
    }

    ProcessOneMs(p_myIRQ);
}
                     
void ProcessFiveMs(Irq *p_myIRQ)
{
    while(tasks.top_five_ms_buffer != tasks.bot_five_ms_buffer)
    {
        ProcessTwoMs(p_myIRQ);
                 
        track(tasks.five_ms_tasks[tasks.bot_five_ms_buffer], p_myIRQ);
        
        tasks.bot_five_ms_buffer++;
        
        tasks.bot_five_ms_buffer %= (NO_CHAN  + 1);

    }

    ProcessTwoMs(p_myIRQ);
}

void ProcessTenMs(Irq *p_myIRQ)
{
    while(tasks.top_ten_ms_buffer != tasks.bot_ten_ms_buffer)
    {
        ProcessFiveMs(p_myIRQ);

        track(tasks.ten_ms_tasks[tasks.bot_ten_ms_buffer], p_myIRQ);
        
        tasks.bot_ten_ms_buffer++;
        
        tasks.bot_ten_ms_buffer %= (NO_CHAN + 1);

    }

    ProcessFiveMs(p_myIRQ);
}



void Process1pps(unsigned int Instruction)
{
// PQ: temperarily remove the 1PPS signal processing, put it back properly later on
}

void AcquireChannel(Irq *p_myIRQ)
{
    unsigned int index_Channel = 0, Break = FALSE;
    int Count;
    long TempLong;

    /* Check if there is a pending task and if so acquire */

    while((index_Channel < NO_CHAN) && (Break == FALSE))
    {
        if(Pending[index_Channel].Flag != FALSE)  /* this channel is assigned with a pending task*/
        {
            // obsolete: Add a bit to the CHANNELS_ACTIVE in DPRAM
            // TempLong = DPRAM[CHANNELS_ACTIVE] & 0x7FFF;
            // TempLong |= (0x1 << Try);
            // DPRAM[CHANNELS_ACTIVE] = TempLong;
			// 
            // PQ: now it is adding a bit to the DPRAM_CHANNELS_ACTIVE_PQ variable
            TempLong = DPRAM_CHANNELS_ACTIVE_PQ & 0x7FFF; //long: 40 bits
            TempLong |= (0x1 << index_Channel);
            DPRAM_CHANNELS_ACTIVE_PQ = TempLong;

            make_active_channels_matrix((int)TempLong, p_myIRQ);

			// New channel to lock
            if(Pending[index_Channel].Flag == NEW) 
            {
                track_initialize(index_Channel,
                                 Pending[index_Channel].Doppler,
                                 Pending[index_Channel].System,
                                 Pending[index_Channel].SV,
                                 Pending[index_Channel].GPSONGLNFlag, p_myIRQ);

            }
            else if(Pending[index_Channel].Flag == NEXT_FREQ_BIN) 
            {
                track_initialize(index_Channel,
                                 Pending[index_Channel].Doppler+freq_bin[freq_bin_index],
                                 Pending[index_Channel].System,
                                 Pending[index_Channel].SV,
                                 Pending[index_Channel].GPSONGLNFlag, p_myIRQ);

            }
            else if( Pending[index_Channel].Flag == RELOCK )
            {
                /* Channel to be relocked.  All information was setup in */
                /* return_to_one so start integration timer and the      */
                /* channel should continue whence it left.               */
				// ???????? PQ: why return_to_one function is not called.
                Start_Integration(index_Channel,PRIMARY,p_myIRQ);
                tracking_state[index_Channel] = STATE_0;

            }
            /* Channel will now have a 1ms task */
            channel_state_log[index_Channel] = 1;
            /* Make a new count of 1ms tasks */
            Count = 0;
            tasks.NumberOneMs = 0;
            tasks.NumberTwoMs = 0;
            while((Count < NO_CHAN) && (active_channels[Count] != ACTIVE_CHANNELS_END))
            {
                if(channel_state_log[active_channels[Count]] == 1)
                    tasks.NumberOneMs++;
                if(channel_state_log[active_channels[Count]] == 2)
                    tasks.NumberTwoMs++;
                Count++;
            }





            Pending[index_Channel].Flag = FALSE;

            Break = TRUE;
        }
        index_Channel++;
    }
}

void make_active_channels_matrix(int channels, Irq *p_myIRQ)
{
    int i,j=0;

    DISABLEINTERRUPTS;

    for(i=0; i<NO_CHAN; i++)
    {
        if(((channels >> i) & 0x1) == 1)  /* Channel active */
        {
            active_channels[j] = i;
            j++;
        }
    }
    active_channels[j] = ACTIVE_CHANNELS_END;  /* ACTIVE_CHANNELS_END is a marker for the end of the matrix */

    ENABLEINTERRUPTS;
}

void load_task_structure(int chn)
{
    switch(tracking_state[chn])
    {
        case UNUSED:
            break;

        case STATE_0:
        case STATE_1:   /* Correlate once per millisecond until energy is detected */
        case STATE_2:   /* start DFLL & DLL and pull-in */
        case STATE_3:   /* start DPLL & DLL and pull-in */
        case STATE_4 :  
        case STATE_5 :         
            tasks.one_ms_tasks[tasks.top_one_ms_buffer] = chn;
            tasks.top_one_ms_buffer ++;
            tasks.top_one_ms_buffer %= (NO_CHAN + 1);

            channel_state_log[chn] = 1;  /* Channel at 1ms */

            break;

        case STATE_5A:    
            tasks.two_ms_tasks[tasks.top_two_ms_buffer] = chn;
            tasks.top_two_ms_buffer ++;
            tasks.top_two_ms_buffer %= (NO_CHAN + 1);

            channel_state_log[chn] = 2;  /* Channel at 2ms */

            break;

        case STATE_5B:    
            tasks.five_ms_tasks[tasks.top_five_ms_buffer] = chn;
            tasks.top_five_ms_buffer ++;
            tasks.top_five_ms_buffer %= (NO_CHAN + 1);

            channel_state_log[chn] = 5;  /* Channel at 5ms */

            break;

        case STATE_5C:    
        case STATE_5CA:
        case STATE_5D:
        case STATE_5E:
        case STATE_5F:
        case STATE_6:   
        case STATE_7:
        case STATE_7A:
        case STATE_7B:
        case STATE_7C:
        case STATE_7D:
        case STATE_7E:
        case STATE_7F:
            tasks.ten_ms_tasks[tasks.top_ten_ms_buffer] = chn;
            tasks.top_ten_ms_buffer ++;
            tasks.top_ten_ms_buffer %= (NO_CHAN + 1);

            channel_state_log[chn] = 10;  /* Channel at 10ms */

            break;
    }
} 

// end of ...

void code_second_order(int chn,float phase_error, Irq *p_myIRQ)
{   
    long carrier_nominal_freq = 0L; /* Difference between the nominal IF frequency and */
                                    /* the actual IF output ie the DOPPLER             */
                               
    unsigned long code_freq = 0L;   /* code chipping frequency */
    float decimal_code_freq = 0.0F;
    
    float code_carr_ratio = 0.0F;   /* Ratio between carrier RF freq. and PRN chipping freq. */
    
    float aid_freq;  /* Estimate of code Doppler from carrier */
    
    float phi_k;     /* filtered code phase error in degrees */

    float v;         /* filter intermediate variable */

    float conv;

    /*************************************************************************/
    /*                                                                       */
    /* Need to scale the carrier Doppler by the ratio of the code chipping   */
    /* frequency and the carrier frequency. In order to do this the nominal  */
    /* IF is subtracted from the carrier NCO output frequency (note          */
    /* calculation is in units of 'NCO words'). This provides the carrier    */
    /* Doppler. The ratio between the chipping rate and the RF frequency is  */
    /* calculated and then the estimate of code Doppler 'aid-frequency' is   */
    /* calculated.                                                           */
    /*                                                                       */
    /*************************************************************************/

    switch(sys[chn])
    {
        case GLONASS:
            /* The carrier control word consists of two parts to calculate the */
            /* absolute carrier Doppler determine the nominal value of the two */
            /* words and subtract from the actual words (scaling correctly)    */
		    //cio << bold << "disabled GLONASS, warning yrtyrd\n" << normal << endl;
            
            carrier_nominal_freq  = carrier_low[L1][chn] - GLN_L1_WORD[channel_num[chn]];
            carrier_nominal_freq += (carrier_top[L1][chn] - GLN_L1_TOP[channel_num[chn]]) * TWO_POWER_32;
            
            /* Each GLONASS satellite transmits at a different carrier freq so */
            /* to calculate the code/carrier ratio. This is then multiplied by */
            /* the code factor of 12 because of hardware differences between   */
            /* the code and carrier NCO                                        */
            /* ie to represent a frequency of 1Hz on the carrier NCO a word    */
            /* 12 times greater than is necessary on the code NCO to produce   */
            /* the same frequency                                              */
                                                     
            code_carr_ratio = 0.511F/((1602.0F + 0.625F*(float)channel_num[chn])*12.0F);
            /* To account for tracking the image as the signal has folded at baseband*/

            if(channel_num[chn] < GLONASS_FOLDOVER_CHANNEL_L1)/*GB assuming L1 only operation in this function*/
                code_carr_ratio *= -1.0;

            /* code frequency word assuming no Doppler */
            code_freq = GLONASS_NOMINAL_CODE_WORD_INT;
            decimal_code_freq = GLONASS_NOMINAL_CODE_WORD_FLOAT;

            break;

        case GPS:
        case INMARSAT: /* JC 30/10/97 INM */
            /* The carrier control word consists of two parts to calculate the */
            /* absolute carrier Doppler determine the nominal value of the two */
            /* words and subtract from the actual words (scaling correctly)    */
            carrier_nominal_freq  = carrier_low[L1][chn] - GPS_L1_WORD;
            carrier_nominal_freq += (carrier_top[L1][chn] - GPS_L1_TOP) * TWO_POWER_32; // either + or -
            
            /* GPS carrier freq is 1540 times the code factor of 12 because of */
            /* hardware differences between the code and carrier NCO           */
            /* ie to represent a frequency of 1Hz on the carrier NCO a word    */
            /* 12 times greater than is necessary on the code NCO to produce   */
            /* the same frequency                                              */
            code_carr_ratio = 1.0F/(1540.0F*12.0F);
            
            /* GPS code frequency word assuming no Doppler                     */
            /*
            code_freq = 219687577L;
            decimal_code_freq = 0.1904F;
            */

            code_freq = GPS_NOMINAL_CODE_WORD_INT;
            decimal_code_freq = GPS_NOMINAL_CODE_WORD_FLOAT;

#ifdef CLOCK_24_MHZ
/*
            code_carr_ratio *= -1;
*/
#endif

            break;
    }


    /* Calculate the estimate of code Doppler from the carrier Doppler */
    aid_freq = ((float)carrier_nominal_freq) * code_carr_ratio; // either + or -

    /* Execute the filter - expressed using the difference operator */

    v = phase_error - r1*w[chn] - r2*x[chn];

    phi_k = c0*v + c1*w[chn] + c2*x[chn];

    /* Now save the differences for next time */
    x[chn] += w[chn];

    w[chn] += v;

    phi_k *= ONE_HZ_WORD_CODE;


    /* Update code NCO frequency word                                            */
    /* New frequency consists of:-                                               */
    /*      aid_freq - estimate of Doppler from carrier Doppler (carrier aiding) */
    /*      code_freq - nominal chipping rate                                    */
    /*      phi_k     - filtered error signal - essentially to allow code carrier*/
    /*                  divergence due to the ionsophere.                        */

    phi_acc[chn] += phi_k;

    /* Need to account for float to int rounding (Hence the 0.5) */
    /*                                                                */
    /* Note:- to avoid numerical truncation the integer & decimal     */
    /*        parts of the nominal code frequency have been separated.*/

    conv = phi_acc[chn] + decimal_code_freq + aid_freq;
    if(conv < 0.0F)
        conv -= 0.5F;
    else
        conv += 0.5F;

    code_freq_word[CA_primary][chn] = code_freq + (long)(conv);

    /* update the hardware with the new frequency */
    Set_Code_NCO(chn,CACODE,code_freq_word[CA_primary][chn], p_myIRQ);

}

float calc_carrier_error(int chn,int sub_chn,int *sign,int band,float T,int USE_INTEGRATED_TOTALS,int type)
{
    int quadrant;   /* I/Q quadrant */
    long i_local;   /* local copy of I punctual */
    long q_local;   /* local copy of Q punctual */
    float phase_error; /* calculated phase error in degrees, return value */
		// PQ 090710 phase_error should be in cycle(or say 2*pi) instead of deg
    int index;      /* Index into the look-up-table */      
    
    if(USE_INTEGRATED_TOTALS == FALSE)
    {
        /* Coded operation - use the correlator totlas returned by the hardware */
        i_local = I_B[sub_chn][chn];  /* copy I punctual */
    }    
    else    
    {
        /* Codeless operation - use the software accumulated correlation totals */
        /* Note:- C/A code is nver integrated only P(1) or P(2) therefore the   */
        /* variable format is                                                   */
        /*                                                                      */
        /*                I/Q_A/B_INT[L1/L2][HW channel number]                 */
        /*                                                                      */
        
        i_local = I_B_INT[band][chn];  /* copy I punctual */
    }    
    if(i_local > 0)
        *sign = 1;
    else    
        *sign = 0;
        
    if(USE_INTEGRATED_TOTALS == FALSE)
    {
        q_local = Q_B[sub_chn][chn];  /* copy Q punctual */
    }
    else
    {
        q_local = Q_B_INT[band][chn];  /* copy Q punctual */
    }
        
     
    /**********************************************************/
    /*                                                        */
    /* By checking the sign of the I and Q correlators the    */
    /* correct quadrant for the arctangent is determined.     */
    /*                                                        */
    /**********************************************************/    
    // type is function input argument
    if(type == NINETY_DEG)
    {
        quadrant=0;
        if(q_local < 0)
        {
            q_local = -q_local;
            quadrant++;
        }
        if(i_local < 0)
        {
            i_local = -i_local;
            quadrant--;
        }
    }
    else // type is of NINETY_DEG or ONE_EIGHTY_DEG
    {   
        /* Make I and Q both positive while also storing the quadrant */
        /* information.                                               */
        
        quadrant = 0;
    
        if(q_local < 0)
        {
            q_local = -q_local;
            quadrant = 1;
        }
        
        if(i_local < 0)
        {
            i_local = -i_local;
            if(quadrant == 0)
                quadrant = 3;
            else             
                quadrant = 2;
        }
    }    
     
    if((i_local==0)&&(q_local==0))  /* Avoid divide by zero */
        phase_error=0.0F;
    else
    {   
        /* Use the look-up-table to calculate the phase error */
        if (q_local > i_local)
        {  //  0<i_local/q_local<1, map it to index 0 to 499 in atn[500]
            index = (int)(0.5F + (float)(i_local)*500.0F/(float)(q_local) ) - 1;
            if(index < 0)
                phase_error = 0.0F;
            else    
                phase_error = 0.25F - atn[index]; // atn(x) = 90deg-atn(1/x)
        }    
        else
        {
            index = (int)(0.5F + (float)(q_local)*500.0F/(float)(i_local)) - 1;
            if(index < 0)
                phase_error = 0.0F;
            else    
                phase_error = atn[index];
        }    
    } 
 
    if(type == NINETY_DEG)
    {
		//				  			quadrant==0  for example phase_error = 0.1rad
		// -------------- 0 --------------------
		//							quadrant!=0
        if(quadrant!=0) /* Correct the phase according to the quadrant */
            phase_error *= -1.0F;
    }
    else
    {

		//	quadrant==3	  			quadrant==0  for example phase_error = 0.1rad
		// -------------- 0 --------------------
		//	quadrant==2				quadrant==1

        /* Extend the range of the arctangent output by using the */
        /* quadrant information.                                  */
        
        switch(quadrant)
        {
            case 0:
                break;
            case 1:
                phase_error *= -1.0F;
                break;
            case 2:
                phase_error -= 0.5F;        
                break;
            case 3:
                phase_error = 0.5F - phase_error;        
                break;
        }        
    
    }        

    /************************************************************************/
    /*                                                                      */
    /* Filter the absolute value of the phase error in order to detect      */
    /* carrier lock.                                                        */
    /*                                                                      */
    /*              Bandwdith of all filters 2.5Hz                          */
    /************************************************************************/
    if(T == 0.001F)
    {
		//float temp = fabsPQ(phase_error);
        filtered_carr_error[band][chn] =   0.99F * filtered_carr_error[band][chn]
                                         + 0.01F * fabsPQ(phase_error); //PQ:it was fabsf
    }
    else
    {
        if(T == 0.1F)
        {
            if(tracking_state[chn] >= STATE_7E)
            {
                filtered_carr_error[band][chn] =   0.333F * filtered_carr_error[band][chn]
                                                 + 0.667F * fabsPQ(phase_error); //PQ:it was fabsf
            }                                     

        }
        else
        {
            filtered_carr_error[band][chn] =   0.905F * filtered_carr_error[band][chn]
                                             + 0.095F * fabsPQ(phase_error); //PQ:it was fabsf
        }                                             
    }
	if (band == L2)
	{
		debug_filtered_carr_error[debug_filtered_carr_error_pnt] = filtered_carr_error[band][chn];
		debug_phase_error1[debug_filtered_carr_error_pnt] = phase_error;
		debug_filtered_carr_error_pnt ++;
		debug_filtered_carr_error_pnt = debug_filtered_carr_error_pnt%200;
	}

    return(phase_error);        
}

void carrier_third_order(int chn,float phase_error_deg,float loop_gain, Irq *p_myIRQ)
{                                                                         
    /* input phase after amplification by the loop gain                              */
    /* NOTE:- non-unity loop gain is used during aquisition so that the coeeficients */
    /*        of the filter do not need modifying when the bandwidth is narrowed     */
    float phase_k;

    float phi_k;   /* filtered carrier phase error in degrees */

    /*************************************************************************/
    /*                                                                       */
    /*                           Function start                              */
    /*                                                                       */
    /*************************************************************************/
    
    phase_k = phase_error_deg * loop_gain;  /* Amplify error signal */

    /* Exectute 3rd order carrier IIR filter */

    phi_k =    phase_k             * CARRIER_IIR_B[0]
             - carrier_z[0][chn]   * CARRIER_IIR_B[1]
             + carrier_z[1][chn]   * CARRIER_IIR_B[2]
             - carrier_phi[0][chn] * CARRIER_IIR_A[0]
             + carrier_phi[1][chn] * CARRIER_IIR_A[1]
             + carrier_phi[2][chn] * CARRIER_IIR_A[2];


    /* shift outpus and inputs ready for when the filter is next called */                                                   
    carrier_phi[2][chn] = carrier_phi[1][chn];
    carrier_phi[1][chn] = carrier_phi[0][chn];            
    carrier_phi[0][chn] = phi_k;                       
    
    carrier_z[1][chn] = carrier_z[0][chn];    
    carrier_z[0][chn] = phase_k;

    /* Convert from degrees to frequency word */                               
    phi_k *= ONE_HZ_WORD_CARR/360.0F;
    
    /* update carrier NCO frequency word 0.5 is to account for float to int rounding */

    if(phi_k < 0.0F)
        phi_k -= 0.5F;
    else
        phi_k += 0.5F;

    carrier_low[L1][chn] += ((long)(phi_k));

    /* Now need to determine whether the bottom word of the NCO control will overflow */
    /* and it is necessary to update the top word                                     */
    carrier_word_wrap(chn,L1,FALSE);
        
    /* update the hardware with the new frequency */ 
    Set_Carrier_NCO(chn,L1CARRIER,carrier_top[L1][chn],carrier_low[L1][chn], p_myIRQ);
    
}

void detect_edges(int chn, Irq *p_myIRQ)
{
    float carr_phase_error;
    float code_phase_error;
    int sign;
    int bit_length;
                               
    /* Read the correlation value from hardware */                               
    read_correlator_totals(chn,CA_primary,FINE);  
    
    /* Execute the carrier discriminator */
    carr_phase_error = calc_carrier_error(chn,CA_primary,&sign,L1,0.001F,FALSE,NINETY_DEG);
    
    /* Execute the 3rd order carrier filter & apply the new frequency to the NCO */
    carrier_third_order(chn,carr_phase_error*360.0F,1.0, p_myIRQ);

    /* Calculate the code tracking error from the non-coherent dot product discriminator */          
    code_phase_error = dot_product_discriminator(CA_primary,chn,FALSE);
    
    /* Execute the second order carrier aided code filter */
    code_second_order(chn,code_phase_error, p_myIRQ);
    
    /* Build up a history of the sign of the 1ms correlation totals */ 
    MS_data_store[chn] <<= 1;                          
    MS_data_store[chn] += sign;
    
    /* Compare bit history with 00000000001111111111 */
    if((MS_data_store[chn] & 0xfffff) == 0x003ff)
    {
        /* If the bit pattern occured assume a bit edge has been detected */
        
        if(bit_sync[chn] == FALSE)
        {
            switch(sys[chn])
            {
                case GLONASS:
                    interrupt_number[chn] = 0;  /* end of bit for GLONASS */
                    bit_sync[chn] = FOUND_ONE;
                    break;
                
                case GPS:    
                    interrupt_number[chn] = 10;  /* middle of bit for GPS */   
                    /* For GPS the middle of the bit occured */
                    bit_sync[chn] = FOUND_ONE;  
                    break;
                
                case INMARSAT:  /* baud rate not known */
                default:
                    break;    
            }
            MS_data_store[chn] = 0xffffffff;
        }
        else
        {
            switch(sys[chn])
            {
                case GLONASS: 
                    if(interrupt_number[chn] == 0)
                         bit_sync[chn] = TRUE;
                    else
                    {   
                        /**************************************************/
                        /*                                                */
                        /* Allow a timeout in case the first data edge    */
                        /* was incorrect                                  */
                        /*                                                */
                        /**************************************************/
                        if(task_counter[chn] >= 120)
                        {
                            bit_sync[chn] = FALSE;
                            task_counter[chn] = 0;
                            code_lock_thres[CA_primary][chn] = 100000000L;
                        }
                    
                    }     
                    break;
                
                case GPS:    
                    if(interrupt_number[chn] == 10)
                        bit_sync[chn] = FOUND_MIDDLE;  
                    else
                    {    
                        /**************************************************/
                        /*                                                */
                        /* Allow a timeout in case the first data edge    */
                        /* was incorrect                                  */
                        /*                                                */
                        /* Note GPS data edges do not occur as frequently */
                        /*      as for GLONASS.                           */
                        /*                                                */
                        /**************************************************/
                        if(task_counter[chn] >= 1320)
                        {
                            bit_sync[chn] = FALSE;
                            task_counter[chn] = 0;
                            code_lock_thres[CA_primary][chn] = 100000000L;
                        }
                    }
                    break;
                
                case INMARSAT:  /* baud rate not known */
                default:
                    break;    
            }
        }    
    }             
    

    task_counter[chn]++;
    
    if( (task_counter[chn] % 60) == 0)
    {
        /* Check for code lock by accucmualting 60 (I^2 + Q^2) */
        /* and comparing them with a threshold.                */
        
        if(code_lock_thres[CA_primary][chn] < STATE_3_THRES[sys[chn]])
		{ // code_lock_thres[CA_primary][chn] is updated in function dot_product_discriminator() earlier.
			cio << "Detection Edge fail Code" << code_lock_thres[CA_primary][chn] << "<" << STATE_3_THRES[sys[chn]] << normal << endl; cio << noshowcursor << flush;
            return_to_one(chn, p_myIRQ);     /* Code has unlocked so reacquire */
        }   
        code_lock_thres[CA_primary][chn] = 0;    
    }
                                
    if(filtered_carr_error[L1][chn] < PHASE_LOCK_THRES_HYST)
    {                                
        if( (bit_sync[chn] == TRUE) 
        || ((bit_sync[chn] == FOUND_MIDDLE) && (interrupt_number[chn] == 0) ) )
        {
            /* At the end of the bit for both GPS and GLONASS */
            
            /* FORCE 1 MS TASKS INTERFERENCE WORK */
			cio << "Detect Edge Pass " << filtered_carr_error[L1][chn] << "<" << PHASE_LOCK_THRES_HYST << normal << endl; cio << noshowcursor << flush;
            tracking_state[chn] = STATE_5;
            task_counter[chn] = 0;
            sign_store[chn] = 0;
            bit_counter[chn] = 0;
            num_valid_bits[chn] = 0;
        }
    }
    else
    {
        /* Carrier has unlocked */
	    cio << "Detection Edge fail Phase" << filtered_carr_error[L1][chn] << "<" << PHASE_LOCK_THRES_HYST << normal << endl; cio << noshowcursor << flush;
        return_to_one(chn, p_myIRQ);

    }    
    
    if(sys[chn] == GPS)
        bit_length = 20;
    else    
        bit_length = 10;    
        
    interrupt_number[chn]++;
    interrupt_number[chn]%=bit_length;
}
 
float dot_product_discriminator(int sub_chn, int chn,int USE_INTEGRATED_TOTALS)
{                        
    long denom;
    float phase_offset;
    int band = L1;

    /* I^2 + Q^2 (punctual) */
    
    if(USE_INTEGRATED_TOTALS == FALSE)
    {
        denom =   I_B[sub_chn][chn]*I_B[sub_chn][chn]
                + Q_B[sub_chn][chn]*Q_B[sub_chn][chn];

        code_lock_thres[sub_chn][chn] += denom;
        
        /* sum the raw I^2 + Q^2 sums to return to PC - gives CNR */
        if(CNR_sum_count[sub_chn][chn] < 100)
        {
            CNR_sum[sub_chn][chn] += (denom/10L);
            CNR_sum_count[sub_chn][chn]++;
        }
        
        
        /* Filter the denominator if locked */
        if(    ((sub_chn == CA_primary) && (tracking_state[chn] >= STATE_6 ))   /* If C/A code & locked */
            || ((sub_chn != CA_primary) && (tracking_state[chn] >= STATE_7F)) ) /* If P(1) or P(2) & locked */
        {   
            /* When in STATE_6 the C/A code observables are used */
            /* to produce a better estimate of the normalization */
            /* for the dot-product discriminator filter the      */
            /* I^2+Q^2 sums. Integration time will be 10ms, the  */
            /* following first order IIR filter has a 0.1Hz BW   */
                
            filtered_code_normalization[sub_chn][chn] =   0.99565F*filtered_code_normalization[sub_chn][chn]
                                                        + 0.00435F*(float)denom;
                
            denom = (long)(filtered_code_normalization[sub_chn][chn] + 0.5F);
        }
        else
        {
            /* Otherwise initialize the filter */
            filtered_code_normalization[sub_chn][chn] = (float)denom;
        } 
    }
    else            
    {
        /* USE_INTEGRATED_TOTALS is never set for C/A code tracking */
        if(sub_chn == P_L1)
            band = L1;
        else    
            band = L2; 
            
        denom =   I_B_INT[band][chn]*I_B_INT[band][chn]
                + Q_B_INT[band][chn]*Q_B_INT[band][chn];

        /* right shift to prevent summation from overflowing */
        code_lock_thres[sub_chn][chn] += (denom>>4);

        /* sum the raw I^2 + Q^2 sums to return to PC - gives CNR */
        if(CNR_sum_count[sub_chn][chn] < 10)
        {
            CNR_sum[sub_chn][chn] += (denom/10L);
            CNR_sum_count[sub_chn][chn]++;
        }
        
        if(tracking_state[chn] >= STATE_7F)
        {        
            /* 100ms integration therefore the filter coefficents have been */
            /* changed to give a 0.1Hz bandwidth.                           */
            
            filtered_code_normalization[sub_chn][chn] =   0.957F*filtered_code_normalization[sub_chn][chn]
                                                        + 0.043F*(float)denom;
                
            denom = (long)(filtered_code_normalization[sub_chn][chn] + 0.5F);
        } 
        else
        {
            filtered_code_normalization[sub_chn][chn] = (float)denom;
        }                
    }

    
    
    if(denom != 0)  /* Avoid divide by zero */
    {   
    
        /* Execute the dot-product discriminator - the 0.0625 is a scale factor */
        /* to account for the difference in scale between E-L & P correlators   */
        /* and produce the error in chips.                                      */
        
        if(USE_INTEGRATED_TOTALS == FALSE)
        {        
            phase_offset  = 0.0625F * (float)(I_B[sub_chn][chn] * I_A[sub_chn][chn]
                                           +  Q_B[sub_chn][chn] * Q_A[sub_chn][chn]);

            /* Removed Q from discriminator */
/*
            phase_offset  = 0.0625F * (float)(I_B[sub_chn][chn] * I_A[sub_chn][chn]);
*/
        }
        else
        {
            phase_offset  = 0.0625F * (float)(I_B_INT[band][chn] * I_A_INT[band][chn]
                                           +  Q_B_INT[band][chn] * Q_A_INT[band][chn]);
        }                                   

        /* Normalize */
        phase_offset /= (float)denom;
        
        /* Limit to +/- 0.5 chips - outside this range the discriminator is */
        /* very non-linear and would increase the loop gain significantly.  */
        /* This could then cause the loops to go unstable.                  */
        
        if(phase_offset > 0.5F)
            phase_offset = 0.5F;

        if(phase_offset < -0.5F)
            phase_offset = -0.5F;

    }
    else
        phase_offset = 0.0F;
        
    return(phase_offset);    
}

void change_integration_period(int chn, Irq *p_myIRQ)
{
    float carr_phase_error;
    int sign;
    float code_phase_error;
    int bit_length;
    float K1,K2,K3,T;
    float f_out;
    float conv;

    /* Read the I and Q C/A correlators */
    read_correlator_totals(chn,CA_primary,FINE);
    
    /* Calculate the carrier tracking error */
    carr_phase_error = calc_carrier_error(chn,CA_primary,&sign,L1,Integration_time[chn],FALSE,NINETY_DEG);

    /*********************************************************************/
    /*                                                                   */
    /* Using a non-coherent dot-product calculate the code phase tracking*/
    /* error - note I(E-l) and Q(E-L) is formed in hardware.             */
    /*                                                                   */
    /*********************************************************************/
                    
    code_phase_error = dot_product_discriminator(CA_primary,chn,FALSE);

    /***********************************************************************/
    /*                                                                     */
    /* Now have the code and carrier discriminator tracking error values   */
    /*                                                                     */
    /*          carr_phase_error & code_phase_error                        */
    /*                                                                     */
    /***********************************************************************/

    /***********************************************************************/
    /*                                                                     */
    /* Want to change the integration period in H/W and modify the tracking*/
    /* filters so that they are still tracking "optimally" without a large */
    /* transient occuring when the integration time (T) is changed.        */
    /* To achieve this intially a filter derived by Raby is used, this has */
    /* a larger acquisition frequency range than the final filters which   */
    /* based on the JPL filter (Stephens & Thomas). In order to switch over*/
    /* to the new filter with a small transients the filter states is      */
    /* achieved by calculating the dynamics by building a a history of the */
    /* filter sums before switching over. The time required to produce     */
    /* sums with sufficient accuracy will depend on the SNR.               */
    /*                                                                     */
    /***********************************************************************/
    if(task_counter[chn] < CHANGE_TO_2ms) /* This constant must be even!! */
    {
        /* Call the "Raby" filter */
        carrier_third_order(chn,carr_phase_error*360.0F,1.0, p_myIRQ);

        /* Build up the carrier tracking sums (in code cycles) */
        carr_filter_state_1_float[L1][chn] += carr_phase_error;
        carr_filter_state_2_float[L1][chn] += carr_filter_state_1_float[L1][chn];

        /* Call the second order carrier aided code filter and update the NCO */

        code_second_order(chn,code_phase_error, p_myIRQ);

        /* Build up the code tracking sums (in code cycles) */
        code_filter_state_1[CA_primary][chn] += code_phase_error;
        code_filter_state_2[CA_primary][chn] += code_filter_state_1[CA_primary][chn];
        
        T = 0.001F;
    }
    else
    {
        code_phase_store[chn] += code_phase_error;
        carr_phase_store[chn] += carr_phase_error;

        switch(task_counter[chn])
        {
            case CHANGE_TO_2ms:
				cio << "CHANGE_TO_2ms" << normal << endl; cio << noshowcursor << flush;
                Change_Dwell_Timing (chn,PRIMARY,TWO_MS,sys[chn], p_myIRQ);     /* Change to 2ms */
                tracking_state[chn] = STATE_5A;
                change_carrier_filter(0.001F,0.002F,&carr_phase_error,chn);
                change_code_filter(0.001F,0.002F,&code_phase_error,chn);
                freq_offset[chn] = carrier_low[L1][chn];
                freq_offset_top[chn] = 0;
                carr_phase_store[chn] = 0.0F;
                code_phase_store[chn] = 0.0F;
                Integration_time[chn] = 0.002F;
                bit_count[chn] = 0;
                break;

            case CHANGE_TO_5ms:
				cio << "CHANGE_TO_5ms" << normal << endl; cio << noshowcursor << flush;
                Change_Dwell_Timing (chn,PRIMARY,FIVE_MS,sys[chn], p_myIRQ);    /* Change to 5ms */
                tracking_state[chn] = STATE_5B;
                carr_phase_error = carr_phase_store[chn] / (float)(CHANGE_TO_5ms - CHANGE_TO_2ms);
                change_carrier_filter(0.002F,0.005F,&carr_phase_error,chn);
                code_phase_error = code_phase_store[chn] / (float)(CHANGE_TO_5ms - CHANGE_TO_2ms);

                if(sys[chn] == GLONASS)
				{
                    code_phase_error = 0.0F;
				}
                change_code_filter(0.002F,0.005F,&code_phase_error,chn);
                code_phase_store[chn] = 0.0F;
                carr_phase_store[chn] = 0.0F;
                Integration_time[chn] = 0.005F;
                bit_count[chn] = 0;
                break;

            case CHANGE_TO_10ms:
				cio << "CHANGE_TO_10ms" << normal << endl; cio << noshowcursor << flush;
                Change_Dwell_Timing (chn,PRIMARY,TEN_MS,sys[chn], p_myIRQ);     /* Change to 10ms */
                tracking_state[chn] = STATE_5C;
                carr_phase_error = carr_phase_store[chn] / (float)(CHANGE_TO_10ms - CHANGE_TO_5ms);
                change_carrier_filter(0.005F,0.010F,&carr_phase_error,chn);
                code_phase_error = code_phase_store[chn] / (float)(CHANGE_TO_10ms - CHANGE_TO_5ms);

                /* Originally, this reset of code_phase_error was for GLONASS */
                /* only.  It has since prooved to work better for GPS also.   */
                code_phase_error = 0.0F;

                change_code_filter(0.005F,0.010F,&code_phase_error,chn);
                code_phase_store[chn] = 0.0F;
                carr_phase_store[chn] = 0.0F;
                Integration_time[chn] = 0.010F;
                bit_count[chn] = 0;
                break;

            case CHANGE_TO_POINT_ONE_10ms:
				cio << "CHANGE_TO_POINT_ONE_10ms" << normal << endl; cio << noshowcursor << flush;
                /**************************************************************/
                /*                                                            */
                /* The tracking is now at the final integration time & the    */
                /* code transient should have reduced. Now the spacing can be */
                /* reduced to 0.1 chips.                                      */
                /*                                                            */
                /**************************************************************/
                if(sys[chn] == GPS)
                {
                    /* Remove phase added when changed to 10ms */
                    /*Shift_Code_NCO_Phase (chn,CACODE,3,ASYNC,FALSE);*/

                    /*Shift_Code_NCO_Phase (chn,CACODE,-2,ASYNC,FALSE);*/
/*
                    Shift_Code_NCO_Phase (chn,CACODE,-3*2,ASYNC,FALSE);
                    CA_Lag_Mode[chn] = NARROW_CA+N_E_L+N_2T_SPACING;
*/
                    /* Comment out the next 2 lines to stop change to 0.1 */
/*
                    Set_Lag_Spacing(chn,PRIMARY,FULL+CA_Lag_Mode[chn]);  / 0.1 chip  E-L in H/W /
                    spacing[chn] = POINT_ONE;
*/
                }
                else
                {
                    Shift_Code_NCO_Phase (chn,CACODE,-2*4,ASYNC,FALSE, p_myIRQ);
                    CA_Lag_Mode[chn] = NARROW_CA+N_E_L+N_4T_SPACING;

                    Set_Lag_Spacing(chn,PRIMARY,FULL+CA_Lag_Mode[chn], p_myIRQ);  /* 0.1 chip  E-L in H/W */
                    spacing[chn] = POINT_ONE;
                }

                /**************************************************************/
                /*                                                            */
                /* Because the hardware can generate a minimum E-L of 2T where*/
                /* T is the sample clock it is possible to reduce the GLONASS */
                /* spacing to 0.05 chips. For experimental purposes this can  */
                /* be achieved by setting the GLONASS_05 flag to TRUE and the */
                /* firmware will automatically set GLONASS up for this spacing*/
                /*                                                            */
                /**************************************************************/

                /* Stop the bit decision INTERFERENCE WORK */

                if((sys[chn] == GPS) || (GLONASS_05 == FALSE))
                    tracking_state[chn] = STATE_5CA;

                break;

            case CHANGE_TO_POINT_ZERO_FIVE_GLONASS_10ms:
                /**************************************************************/
                /*                                                            */
                /* If the flag has been set to track GLONASS at 0.05 chips    */
                /* between early and late change the spacing.                 */
                /*                                                            */
                /**************************************************************/

                if((sys[chn] == GLONASS) && (GLONASS_05 == TRUE))
                {
		    		//cio << bold << "disabled GLONASS, warning nrdtity\n" << normal << endl;
                    Shift_Code_NCO_Phase (chn,CACODE,-1*4,ASYNC,FALSE, p_myIRQ);
                    CA_Lag_Mode[chn] = NARROW_CA+N_E_L+N_2T_SPACING; /* 0.05 chip  E-L in H/W */
                    Set_Lag_Spacing(chn,PRIMARY,FULL+CA_Lag_Mode[chn], p_myIRQ);
                    spacing[chn] = POINT_ZERO_FIVE;
                    tracking_state[chn] = STATE_5CA;
                }
                break;

            default:
                /* Update the 3rd order carrier sums */
                carr_filter_state_1_float[L1][chn] += carr_phase_error;
                carr_filter_state_2_float[L1][chn] += carr_filter_state_1_float[L1][chn];

                /* Update the 2nd order  code sum */
                code_filter_state_1[CA_primary][chn] += code_phase_error;

                /* The 3rd order, 2nd integrator is accumulated in */
                /* code_aid_integrated function  */
        }


        /* 10Hz */
        K1 = 0.1741F;
        K2 = 0.01313F;
        K3 = 3.585e-4F;

        /* 15Hz (10ms integration) */
        /*
        K1 = 0.2142F;
        K2 = 0.02208F;
        K3 = 8.655e-4F;
        */
        T = Integration_time[chn];

        /* Execute the "JPL" 3rd order carrier filter */
        f_out = (  K1*carr_phase_error + K2*carr_filter_state_1_float[L1][chn]
                 + K3*carr_filter_state_2_float[L1][chn])/T;

        /* Update the carrier NCO word */
        conv = f_out*ONE_HZ_WORD_CARR;
        if(conv < 0.0F)
            conv -= 0.5F;
        else
            conv += 0.5F;

        carrier_low[L1][chn] = freq_offset[chn] + (long)(conv);
        carrier_low[L1][chn] -= freq_offset_top[chn]*TWO_POWER_32;


        /* Account for the carrier word being in two parts */
        carrier_word_wrap(chn,L1,TRUE);
        
        /* update the carrier hardware with the new frequency */
        Set_Carrier_NCO(chn,L1CARRIER,carrier_top[L1][chn],carrier_low[L1][chn], p_myIRQ);

        /* Call the modified code filter & output  */

        code_aid_integrated(chn,code_phase_error,CA_primary,L1,T, p_myIRQ);

    }
    
    /**************************************************************************/
    /*                                                                        */
    /* Next section calculates the sign of the bit by accumulating the I      */
    /* punctual values and inspecting the sign.                               */
    /*                                                                        */
    /**************************************************************************/
    if(sys[chn] == GPS)
        bit_length = 20;
    else                                
        bit_length = 10;

    sign_store[chn] += I_B[CA_primary][chn];    /* Accumulate I on time */
    
    /* At the end of the bit make the decision */
    if(bit_counter[chn] >= bit_length) // 20 for GPS and 10 for GLONASS, just according to current loop closure ms number
    {
        data_store[chn] <<= 1;
            
        /* If the correlator sum is positive then the bit is a one */
        if(sign_store[chn] > 0)
		{
            data_store[chn] += 1;   /* save in data_store[] for transfer to the host */
			debug_nav_msg[debug_nav_msg_index] = 1;
		}
		debug_nav_msg_index ++;
		if (debug_nav_msg_index == debug_nav_msg_len)
		{
			debug_nav_msg_index = 0;
		}
        sign_store[chn] = 0;
        bit_counter[chn] = 0;
        num_valid_bits[chn]++;

        if(num_valid_bits[chn] >= 16) // PQ: why ?????????????????????????????????
        {
            /* The raw nav bits are NOT written to independent */
            /* banks                                           */
            /*GB - no need for IGEX to write these out so disable
            write_to_dual_port(chn,0,RAW_NAV_BITS,0);           
            */
            num_valid_bits[chn] = 0;
        }


        if(sys[chn] == GPS)
        {
            if(tracking_state[chn] == STATE_5CA)
                gps_preamble(chn);
            else
                if(tracking_state[chn] > STATE_5CA)
                    get_gps_time(chn);
        }
        else
        {
            if((tracking_state[chn] == STATE_5CA) || (tracking_state[chn] == STATE_5D))
                glonass_preamble(chn);
            else
            {
                if(tracking_state[chn] >= STATE_5E)
                    get_glonass_time(chn);
            }
        }

/*
        if(tracking_state[chn] == STATE_5CA)
        {
            if(sys[chn] == GPS)
                gps_preamble(chn);
            else
                glonass_preamble(chn);
        }
        else
        {
            if(tracking_state[chn] > STATE_5CA)
            {
                if(sys[chn] == GPS)
                    get_gps_time(chn);
                else
                    get_glonass_time(chn);
            }            
        }    
*/
    }


    bit_counter[chn] += (int)(T*1000.0F);    /* Update the bit counter by the length of the correlation period */

    task_counter[chn]++;
    
    if(tracking_state[chn] == STATE_5F)
    {
        tracking_state[chn] = STATE_6;
		cio << "enter STATE_6" << normal << endl; cio << noshowcursor << flush;
		SV_occurance[Pending[0].SV-1] = 1;
		
    }

}

void change_carrier_filter(float old_T, float new_T, float *phase_error,int chn)
{
    float K1,K2,K3,T;
    float phi[3];

    if(old_T == 0.001F)
    {
        K1 = 0.02719F;
        K2 = 2.562e-4F;
        K3 = 8.084e-7F;
    }
    else
    {
        /* 10Hz */
        K1 = 0.1741F;
        K2 = 0.01313F;
        K3 = 3.585e-4F;

        /* 15Hz (10ms integration) */
        /*
        K1 = 0.2142F;
        K2 = 0.02208F;
        K3 = 8.655e-4F;
        */
    }

    T = old_T;

    /* Calculate the dynamics  */
    phi[2] = (*phase_error)*K3/(T*T*T);
    phi[1] = ( K3*carr_filter_state_1_float[L1][chn] + K2*(*phase_error) ) / ( T*T );
    phi[0] = ( K3*carr_filter_state_2_float[L1][chn] - T*T*phi[1]*0.5F
         - T*T*T*phi[2]/6.0F + K1*(*phase_error)
         + K2*carr_filter_state_1_float[L1][chn])/T;

    /* change the integration period */
    T  = new_T;

    if(old_T == 0.001F)
    {
        /* 10Hz */
        K1 = 0.1741F;
        K2 = 0.01313F;
        K3 = 3.585e-4F;

        /* 15Hz (10ms integration) */
        /*
        K1 = 0.2142F;
        K2 = 0.02208F;
        K3 = 8.655e-4F;
        */
    }

    /* Calculate the new filer states based on the dynamics & T */
    *phase_error        = T*T*T*phi[2] / K3;
    carr_filter_state_1_float[L1][chn] = (T*T*phi[1] - K2*(*phase_error)) / K3;
    carr_filter_state_2_float[L1][chn] = (T*phi[0] + T*T*phi[1]*0.5F + T*T*T*phi[2]/6.0F
                            - K1*(*phase_error) - K2*carr_filter_state_1_float[L1][chn]) / K3;
}

void change_code_filter(float old_T, float new_T, float *phase_error,int chn)
{
    float K1,K2,K3,T;
    float phi[3];

    if(old_T == 0.001F)
    {
        K1 = 0.01406F;
        K2 = 6.698e-5F;
        K3 = 1.067e-7F;    
    }
    else
    {   
        K1 = 0.02729F;
        K2 = 2.562e-4F;
        K3 = 8.084e-7F;
    }

    T = old_T;

    /* Calculate the dynamics  */
    phi[2] = (*phase_error)*K3/(T*T*T);
    phi[1] = ( K3*code_filter_state_1[CA_primary][chn] + K2*(*phase_error) ) / ( T*T );
    phi[0] = ( K3*code_filter_state_2[CA_primary][chn] - T*T*phi[1]*0.5F
               - T*T*T*phi[2]/6.0F + K1*(*phase_error)
               + K2*code_filter_state_1[CA_primary][chn])/T;
               
    /* change the integration period */
    T  = new_T;

    if(old_T == 0.001F)
    {
       K1 = 0.02729F;
       K2 = 2.562e-4F;
       K3 = 8.084e-7F;
    }

    /* Calculate the new filer states based on the dynamics & T */
    *phase_error        = T*T*T*phi[2] / K3;
    code_filter_state_1[CA_primary][chn] = (T*T*phi[1] - K2*(*phase_error)) / K3;
    code_filter_state_2[CA_primary][chn] = (T*phi[0] + T*T*phi[1]*0.5F + T*T*T*phi[2]/6.0F
                                           - K1*(*phase_error) - K2*code_filter_state_1[CA_primary][chn]) / K3;
}

/****************************************************************************/
/*                                                                          */
/* Function Name:   gps_preamble()                                          */
/*                                                                          */
/* Author:  Stuart Riley                                                    */
/*                                                                          */
/* Version: 1.0     Date:   6/12/94                                         */
/*                                                                          */
/* Description: effeciently detect the GPS preamble                         */
/*              - developed from the Leeds GNSS TMS code                    */
/*                                                                          */
/* The original Leeds code used to have several returns in the function     */
/* this speeded operation as additional if statements did not have to be    */
/* processed - due to Alcatel Espace coding rules this can no longer be     */
/* done.                                                                    */
/*                                                                          */
/* Data IN:  chn     - hardware channel number                              */
/*                                                                          */
/* Data Out: Nothing                                                        */
/*                                                                          */
/* Globals changed:                                                         */
/*                                                                          */
/* Calls:                                                                   */
/*                                                                          */
/****************************************************************************/

void gps_preamble(int chn)
{
    long data_in;    /* local copy of the binary data message */

    data_in = data_store[chn]&255; /* Mask off bottom 8 bits */
    
    
    /*************************************************************************/
    /*                                                                       */
    /* A bit counter is used to determine whether to check for the pre-amble */
    /* or not - when a possible occurance of the preamble is detected the    */
    /* variable is set to 8 as the eighth bit has been received.             */
    /*                                                                       */
    /*                                                                       */
    /*************************************************************************/
    
    if(bit_count[chn]<9 && (data_in==0x8b || data_in==0x74))
    {    
        /* 0x8b is the preamble                   */
        /* 0x74 is its inverse                    */
        /* Found possible pre-amble, check parity */

        if(data_in==0x74)
		{
            inverted |= (1 << chn);   /* preamble is inverted */
        }                        
        /*********************************************************************/
        /*                                                                   */
        /* 0x8b/0x74 may occur randomly quite frequently - if a possible     */
        /* pre-amble is detected, this function checks the parity of the word*/
        /* to confirm it is the preamble.                                    */
        /*                                                                   */
        /* To spread the workload a variable parity_check[] is formed from   */
        /* the received bit (updated as each bit is received), and then      */
        /* compared against the received six parity bits.                    */
        /*                                                                   */
        /* The preamble (or its inverse) has been detected and so the        */
        /* contribution to parity_check has been precalculated as 0x12.      */
        /*                                                                   */
        /*********************************************************************/
        
        parity_check[chn]=0x12; 
        bit_count[chn]=8;       /* set bit counter to number of current bit */
    }
                      
    /*************************************************************************/
    /*                                                                       */
    /*      For bits 9 to 24 inclusive update the parity_check variable      */
    /*                                                                       */
    /*************************************************************************/
    
    if(bit_count[chn]>=9 && bit_count[chn]<=24)
    {
        if(((inverted >> chn)&1) == 1) /* If the received data is inverted correct it */
            data_in=~data_in;
                             
        /*********************************************************************/
        /*                                                                   */
        /* This is quite complex and so is not documented here instead see   */
        /* the detailed design document.                                     */
        /*                                                                   */
        /*********************************************************************/
        
        parity_check[chn]^= ( (data_in&1) == 1) ? table[bit_count[chn]-1] : 0;
    }
    
    /*************************************************************************/
    /*                                                                       */
    /* When all 30 bits (one GPS word) have been received the parity_check   */
    /* variable is compared against the received parity bits (25-30) if they */
    /* agree it is assumed that the preamble has been found - otherwise it is*/
    /* assumed that the preamble occured randomly in the received data stream*/
    /* and the preamble search is reinitiated.                               */
    /*                                                                       */
    /*************************************************************************/
        
    if(bit_count[chn] == 30)
    {
        if(((inverted >> chn)&1) == 1) /* If the received data is inverted correct it */
            data_in = ~data_in;

        /* compare the six received parity bits with the parity calculated */
        /* from the data stream                                            */
        
        if( (data_in&0x3f) == (parity_check[chn]&0x3f) )
        {   
            /*****************************************************************/
            /*                                                               */
            /* Once the preamble has been found the time is extracted from   */
            /* the data message. The inital value of the parity table depends*/
            /* on status of D29 and D30 from this word.                      */
            /*                                                               */
            /* First check the sign of D29 and set the parity_check          */
            /*                                                               */
            /*****************************************************************/
            
            if((data_in>>1)&1 == 1) 
                parity_check[chn] = 0x29;
            else
                parity_check[chn] = 0;
                                    
            /* Check the sign of D30 */                                    
            if(data_in&1==1)
                parity_check[chn] += 0x16;
                
            /* D30 also neds storing so that the `source data bits' can be */
            /* recovered from the transmitted data bits.                   */
            if((data_in&1) == 1)
            {
                /* Make the bit corresponding to chn 1 */
                d30 |= (1 << chn);
            }
            else
            {
                /* Make the bit corresponding to chn 0 */
                d30 &= (0xffffffff - ( 1 << chn)); 
            }                    


    		cio << "Preamble OK, enter STATE_5D. inverted=" << inverted << normal << endl; cio << noshowcursor << flush;
            tracking_state[chn] = STATE_5D;    /* Found the preamble go to appropriate state*/
            bit_count[chn] = 0;       /* about to read the next word to extract the time */
        }
        else
        {                       
            inverted &= (0xffffffff - ( 1 << chn));  /* reset inverted */
            bit_count[chn]=0;   /* Failed parity test */
        }
    }
    else
    {
        /* Increment the bit counter */
        if(bit_count[chn]>=8)
            bit_count[chn]++;                                       
    }        
        
}                /********* End of:-  gps_preamble() **********/

/****************************************************************************/
/*                                                                          */
/* Function Name:   get_gps_time()                                          */
/*                                                                          */
/* Author:  Stuart Riley                                                    */
/*                                                                          */
/* Version: 1.0     Date:   6/12/94                                         */
/*                                                                          */
/* Description: extract the GPS time from the data message                  */
/*              - developed from the Leeds GNSS TMS code                    */
/*                                                                          */
/* Data IN:  chn     - hardware channel number                              */
/*                                                                          */
/* Data Out: Nothing                                                        */
/*                                                                          */
/* Globals changed:                                                         */
/*                                                                          */
/* Calls:                                                                   */
/*                                                                          */
/****************************************************************************/
    
void get_gps_time(int chn)
{
    long word;
    long temp;
    long data_in;

    bit_count[chn]++;
    data_in = data_store[chn]; /* for efficiency make a local copy */
                           
    /* If the data message is inverted correct it */                           
    if(((inverted >> chn)&1) == 1)
        data_in=(~data_in);
                                   
    /*************************************************************************/
    /*                                                                       */
    /* Decode and store (in sys_time[]) the data bits 1-24. During decoding, */
    /* XORing with the previous D30, form the parity from the received data  */
    /* bits.                                                                 */
    /*                                                                       */
    /*************************************************************************/
    
    if(bit_count[chn]<=24)
    {
        temp=(data_in&1)^((d30>>chn)&1);
        parity_check[chn]^= (temp == 1) ? table[bit_count[chn]-1] : 0;
        sys_time[chn]<<=1;
        sys_time[chn]+=temp;
    }   
         
    /*************************************************************************/
    /*                                                                       */
    /* When bits 25-30 have been collected compare the received parity with  */
    /* the calculated parity. If it passes manipulate the received data in   */
    /* order to calculate the time.                                          */
    /*                                                                       */
    /* Note:- This function follows detection of the preamble. The preamble  */
    /*        occurs every six seconds at the beginning of a line of data.   */
    /*        The time can be extracted from the HOW which is in the word    */
    /*        immediately after the word containing the preamble.            */
    /*                                                                       */
    /*************************************************************************/
    
    if(bit_count[chn]==30)
    {   
        /* check the received parity with the calculated parity */
        if( (parity_check[chn]&0x3f) == (data_in&0x3f))
        {                        
            /* Manipulate the received data to give the time */
            /* >>7 dumps:-                                   */
            /*           the momentum/alert flag             */
            /*           synchronization/AS flag             */
            /*           subframe ID                         */
            /*                                               */
            /* When multiplied by 6 gives the time in secs   */
            /* at the start of the next line of data.        */
            /* When this word has been read the time is 1.2s */
            /* into the current line (or time given minus    */
            /* 4.8). Subtract 5 seconds to give the integer  */
            /* seconds and set task_counter to the number of */
            /* milliseconds (200).                           */

            /* At this point the variable sys_time contains  */
            /* all the information to create the time but is */
            /* NOT correctly formated.                       */

            word = sys_time[chn];

            /* Extract the A/S flag to determine whether to attempt */
            /* P code or P-W code acquisition.                      */
            /* If set to 1 (TRUE) then A/S is present.              */

            CODELESS_TRACKING[chn] = (word>>5)&0x1;

            /* The remaining BLOCK I SV (PRN 12) transmits a        */
            /* different flag to BLOCK IIs and cannot invoke A/S    */

            if(satellite == 12)
                CODELESS_TRACKING[chn] = FALSE;

            sys_time[chn] = (((word>>7)&0x1ffff) * 6) - 5;
            
            /* When the time has been extracted (and parity check passed) */
            /* Move on to the final tracking state.                       */
            tracking_state[chn] = STATE_5F;
    		cio << "GPS t OK, enter STATE_5F" << normal << endl; cio << noshowcursor << flush;
			
            /* Need to set this to the correct millisecond !!! */
            task_counter[chn] = 20;

            /* Need to set this to a large number as the lock detect will */
            /* called immediatley in the next state (don't want it to     */
            /* incorrectly fail).                                         */

            code_lock_thres[CA_primary][chn] = 2*STATE_6_THRES[sys[chn]];
        }
        else
        {
            /* Failed parity - search for the preamble again */
    		cio << "GPS t fail" << parity_check[chn] << "!=" << data_in << normal << endl; cio << noshowcursor << flush;
        
            bit_count[chn]=0;
            inverted &= (0xffffffff - ( 1 << chn)); /* reset */
            tracking_state[chn] = STATE_5CA; // change_integration_period 10 ms integration time
        }
    }
}               /********** End of:-   get_gps_time()   **********/

/****************************************************************************/
/*                                                                          */
/* Function Name:   glonass_preamble()                                      */
/*                                                                          */
/* Author:  Stuart Riley                                                    */
/*                                                                          */
/* Version: 1.0     Date:   6/12/94                                         */
/*                                                                          */
/* Description: Detect the GLONASS preamble                                 */
/*                                                                          */
/* Data IN:  chn     - hardware channel number                              */
/*                                                                          */
/* Data Out: Nothing                                                        */
/*                                                                          */
/* Globals changed:                                                         */
/*                                                                          */
/* Calls:                                                                   */
/*                                                                          */
/****************************************************************************/

void glonass_preamble(int chn)
{
    long masked_data;    /* last 30 bits of received data */
    long data_in;        /* Local copy of data bits */

    data_in = data_store[chn];   /* for efficiency make a local copy */
 
    /* If the preamble hasn't been found look for it */ 
    if(tracking_state[chn] == STATE_5CA)
    {
        /*********************************************************************/
        /*                                                                   */
        /* The GLONASS C/A code preamble is 30 bits - it is actually a       */
        /* "post-amble" occurring at the end of the two second line of data. */
        /* Because it is at the end of the line and it is so long the parity */
        /* is not tested - the receiver may have locked half-way through the */
        /* line and therefor the parity check would fail, but the receiver   */
        /* may actually be locked. Additionally the probability of randomly  */
        /* receiving the preamble bit pattern, assuming Gaussian noise, is   */
        /* 1/2^29, with a new bit every 10ms this corresponds to once every  */
        /* 62 days (the software searches for both the preamble and the      */
        /* inverse). However, when the time is extracted the parity is also  */
        /* checked to confirm the receiver is correctly locked and the time  */
        /* tag is correct.                                                   */
        /*                                                                   */
        /*********************************************************************/
        
        masked_data = data_in&0x3fffffff;   /* mask off the last 30 bits */
                                                                           
        /* Check for the GLONASS preamble and its inverse */
                                                                                   
        if((masked_data == GLN_PREAMBLE_30) || (masked_data == NOT_GLN_PREAMBLE_30))
        {
            tracking_state[chn] = STATE_5D;
            
            /* Check is the preamble is inverted */
            if(masked_data == GLN_PREAMBLE_30)
            {
                /* Data is not inverted - set bit to zero */
                inverted &= (0xffffffff - (1 << chn));
            }
            else
            {
                /* Data is inverted - set bit to one */
                inverted |= (1 << chn);
            }    
            
            bit_count[chn]=0;
        }
        else
            bit_count[chn]=-1;
    }
        
    /*************************************************************************/
    /*                                                                       */
    /* The GLONASS data is return-to-zero and differentially encoded.        */
    /* Immediately following the preamble, at the start of the next line of  */
    /* data are 2 zeros, then 8 bits which define the 4 bit line number (once*/
    /* the differential and RTZ coding are accounted for).                   */
    /*                                                                       */
    /* The time only occurs on line 1. Therefore the data is continually     */
    /* checked until line 1 occurs.                                          */
    /*                                                                       */
    /*************************************************************************/
    
    if((bit_count[chn] == 10) && (tracking_state[chn] == STATE_5D))
    {
        /* 10 bits into a line - can therefore check the line number */
        
        if(((inverted >> chn)&1) == 1)
            data_in= ~data_in;  /* correct for data inversions */
                                                    
        /* Mask off the last eight bits (ie the line number) and compare      */
        /* against the known value for line 1 (accounting for RTZ and         */
        /* differential encoding.                                             */        
        
        if( (data_in&255) == 0x56)
        {
            tracking_state[chn] = STATE_5E;
            /*previous &= (0xffffffff - ( 1 << chn));*/  /* zero the bit corresponding to this channel */
            previous |= (1 << chn);
            diff_store[chn]=0;
            bit_count[chn]=11;
            parity_check[0] = glo_parity_matrix[(bit_count[chn]-2)/2];                                                            
        }
        else
        {   
            /* If this is not line 1 go back to checking for the preamble - */
            /* the preamble will occur at the end of this line.             */
            
            tracking_state[chn] = STATE_5CA;
            bit_count[chn]=0;
            inverted &= (0xffffffff - ( 1 << chn)); /* reset */
        }
    }
    else
    {                
        bit_count[chn]++;                                           
    }    
    
}           /********** End of:- glonass_preamble()  **********/
 


/****************************************************************************/
/*                                                                          */
/* Function Name:   get_glonass_time()                                      */
/*                                                                          */
/* Author:  Stuart Riley                                                    */
/*                                                                          */
/* Version: 1.0     Date:   6/12/94                                         */
/*                                                                          */
/* Description: Get the GLONASS time from the message.                      */
/*                                                                          */
/* Data IN:  chn     - hardware channel number                              */
/*                                                                          */
/* Data Out: Nothing                                                        */
/*                                                                          */
/* Globals changed:                                                         */
/*                                                                          */
/* Calls:                                                                   */
/*                                                                          */
/****************************************************************************/

void get_glonass_time(int chn)
{                            
    long data_in;        /* Local copy of data bits */
    long temp;

    data_in = data_store[chn];   /* for efficiency make a local copy */                                                                       
                                                                       
    /************************************************************************/
    /*                                                                      */
    /*         On the odd bit unencode the differential and RTZ             */
    /*                                                                      */
    /************************************************************************/
    
    if((bit_count[chn]&1)==1)
    {   
        if(((inverted >> chn)&1) == TRUE)     /* Correct data if inverted */
            data_in  = (~data_in)&1;  /* also mask off bottom bit */
        else
            data_in &= 1;             /* Mask off bottom bit */
            
        diff_store[chn] <<= 1;
        
        /* Unencode differential and RTZ */
        diff_store[chn] += (~( ((previous >> chn)&1) ^ data_in)) & 1;
        
    }    
    else
    {                                 
        
        /************************************************************************/
        /*                                                                      */
        /*          Form the parity matrix from the received data bits.         */
        /*                                                                      */
        /************************************************************************/
        
        if( (bit_count[chn] > 2) &&  (bit_count[chn] <= 154) && ((diff_store[chn]&1) == 1) )                                                            
            parity_check[chn] ^= glo_parity_matrix[(bit_count[chn]-2)/2];                                                            
        
        /************************************************************************/
        /*                                                                      */
        /* On the even bit store the bit for differential/RTZ unencoding and    */
        /* calculate the time when time has been received.                      */
        /*                                                                      */
        /************************************************************************/
    
        /* Time parameter takes 32 RTZ bits after 8 RTZ bits for the line number*/
        /* and the 2 RTZ bits whicha occur prior to the line no.                */
        if(bit_count[chn] == 32+8+2)
        {                                                                                                 
            /* copy differentially and RTZ unencoded data for speed and clarity */
            temp=diff_store[chn];   
            
            /* time+1 as time will not be incremented until int_no==1000 */
            /*                                                           */
            /* For following calculation see the time definitiaon in the */
            /* GLONASS ICD.                                              */

            sys_time[chn] = 1 + ((temp>>7)&31)*3600 + (temp&127)*30 ;
        }
        
         
        /*********************************************************************/
        /*                                                                   */
        /* At the end of a GLONASS data line - check that the parity agrees  */
        /* with the calculated parity.                                       */
        /*                                                                   */
        /*********************************************************************/
        
        if(bit_count[chn] == 170)
        {
            if(parity_check[chn] == (diff_store[chn]&255))
            {
                /* The calculated parity agrees with the received parity bits */
                tracking_state[chn] = STATE_5F;
                task_counter[chn] = 70;  /* changed from 0 - JC 2/2/96 */
            }
            else
            {
                /* Problem with the parity check so remove it for the moment - JC 24/1/96 */
/*
                / Parity incorrect - search for preamble /
                bit_count[chn] = 0;
                tracking_state[chn] = STATE_5CA;
                inverted &= (0xffffffff - ( 1 << chn));  / reset /
*/
                /* Currently, the parity checking doesn't work. Therefore */
                /* when ignoring the fail, set-up the P-code.             */

                tracking_state[chn] = STATE_5F;  /* ignores the parity fail */
                task_counter[chn] = 70;   /* Set up for P-code */
            }  
        }

        /* correct current bit for data inversions and mask off */
        if(((inverted >> chn)&1) == 1)
            data_in=(~data_in )&1;
        else
            data_in&=1;


        if((data_in) == 1)
        {
            /* Make the bit corresponding to chn 1 */
            previous |= (1 << chn);
        }
        else
        {
            /* Make the bit corresponding to chn 0 */
            previous &= (0xffffffff - ( 1 << chn));
        }
    }

    bit_count[chn]++;                                           
    
}

void code_aid_integrated(int chn,float phase_error,int sub_chn,int band,float T, Irq *p_myIRQ)
{   
    int TempState;

    long carrier_nominal_freq; /* Difference between the nominal IF frequency and */
                               /* the actual IF output ie the DOPPLER             */

    float decimal_code_freq;
    unsigned long code_freq;            /* code chipping frequency */
    
    float code_carr_ratio;    /* Ratio between carrier RF freq. and PRN chipping freq. */
    
    float aid_freq;  /* Estimate of code Doppler from carrier */
    
    float phi_k;   /* filtered code phase error in degrees */

    float K1,K2,K3;
    float K1_X100_XCONV; /* K1 * ONE_HZ_WORD_CODE / T (where T = 0.01s) */
    float K2_X100_XCONV; /* K2 * ONE_HZ_WORD_CODE / T (where T = 0.01s) */
    float K3_X100_XCONV; /* K3 * ONE_HZ_WORD_CODE / T (where T = 0.01s) */
    
    long integer;

    float conv;

    /*************************************************************************/
    /*                                                                       */
    /* Need to scale the carrier Doppler by the ratio of the code chipping   */
    /* frequency and the carrier frequency. In order to do this the nominal  */
    /* IF is subtracted from the carrier NCO output frequency (note          */
    /* calculation is in units of 'NCO words'). This provides the carrier    */
    /* Doppler. The ratio between the chipping rate and the RF frequency is  */
    /* calculated and then the estimate of code Doppler 'aid-frequency' is   */
    /* calculated.                                                           */
    /*                                                                       */
    /*************************************************************************/


    switch(sys[chn])
    {
        case GLONASS:
            /* The carrier control word consists of two parts to calculate the */
            /* absolute carrier Doppler determine the nominal value of the two */
            /* words and subtract from the actual words (scaling correctly)    */

            carrier_nominal_freq  = carrier_low[L1][chn] - GLN_L1_WORD[channel_num[chn]];
            carrier_nominal_freq += (carrier_top[L1][chn] - GLN_L1_TOP[channel_num[chn]]) * TWO_POWER_32;

            /* Each GLONASS satellite transmits at a different carrier freq so */
            /* to calculate the code/carrier ratio. This is then multiplied by */
            /* the code factor of 12 because of hardware differences between   */
            /* the code and carrier NCO                                        */
            /* ie to represent a frequency of 1Hz on the carrier NCO a word    */
            /* 12 times greater than is necessary on the code NCO to produce   */
            /* the same frequency                                              */
            if( (sub_chn == CA_primary) || (sub_chn == CA_secondary) )
            {
                code_carr_ratio = GLONASS_L1_CA_carr_code_ratio[channel_num[chn] - 1];

                /* To account for tracking the image as the signal has folded at baseband*/
                if(channel_num[chn] < GLONASS_FOLDOVER_CHANNEL_L1)/*GB L1 only operation */
                    code_carr_ratio *= -1.0;

                /* code frequency word assuming no Doppler */
                code_freq = GLONASS_NOMINAL_CODE_WORD_INT;
                decimal_code_freq = GLONASS_NOMINAL_CODE_WORD_FLOAT;
            }    
            else    
            {
                /* P code the code has 10 times the dynamics of the C/A */
                code_carr_ratio = GLONASS_L1_P_carr_code_ratio[channel_num[chn] - 1];

                /*GB need to account for different L1 and L2 foldover frequencies */
                if(band == L1)
                {
                    /* To account for tracking the image as the signal has folded at baseband*/
                    if(channel_num[chn] < GLONASS_FOLDOVER_CHANNEL_L1)
                        code_carr_ratio *= -1.0;
                }
                else
                {
                    /* To account for tracking the image as the signal has folded at baseband*/
                    if(channel_num[chn] < GLONASS_FOLDOVER_CHANNEL_L2)
                        code_carr_ratio *= -1.0;
                }

                /* code frequency word assuming no Doppler */
                code_freq = GLONASS_P_NOMINAL_CODE_WORD_INT;
                decimal_code_freq = GLONASS_P_NOMINAL_CODE_WORD_FLOAT;
            }
            break;

        case GPS:    
            /* The carrier control word consists of two parts to calculate the */
            /* absolute carrier Doppler determine the nominal value of the two */
            /* words and subtract from the actual words (scaling correctly)    */

            carrier_nominal_freq  = carrier_low[L1][chn] - GPS_L1_WORD;
            carrier_nominal_freq += (carrier_top[L1][chn] - GPS_L1_TOP) * TWO_POWER_32;
            
            /* GPS carrier freq is 1540 times the code factor of 12 because of */
            /* hardware differences between the code and carrier NCO           */
            /* ie to represent a frequency of 1Hz on the carrier NCO a word    */
            /* 12 times greater than is necessary on the code NCO to produce   */
            /* the same frequency                                              */
            if( (sub_chn == CA_primary) || (sub_chn == CA_secondary) )
            {
                code_carr_ratio = 1.0F/(1540.0F*12.0F);
                /* GPS code frequency word assuming no Doppler */
                code_freq = GPS_NOMINAL_CODE_WORD_INT;
                decimal_code_freq = GPS_NOMINAL_CODE_WORD_FLOAT;
            }    
            else    
            {
                /* P code the code has 10 times the dynamics of the C/A */
                code_carr_ratio = 1.0F/(154.0F*12.0F);
                
                /* GPS code frequency word assuming no Doppler */
                code_freq = GPS_P_NOMINAL_CODE_WORD_INT;
                decimal_code_freq = GPS_P_NOMINAL_CODE_WORD_FLOAT;
            }    

#ifdef CLOCK_24_MHZ
/*
            code_carr_ratio *= -1;
*/
#endif
            break;
    } // end of switch 

    /* Calculate the estimate of code Doppler from the carrier Doppler */
    aid_freq = ((float)carrier_nominal_freq) * code_carr_ratio;
    
    if(    (sub_chn != CA_primary)
        && (    tracking_state[chn] == STATE_7A
             || (tracking_state[chn] < STATE_7D && band == L2))
        && (CODELESS_TRACKING[chn] == FALSE) )   /* Coded operation only - need to check this */
    {
        /* 10Hz loop */
        K1 = 0.1741F;
        K2 = 0.01313F;
        K3 = 3.585e-4F;

#ifdef CLOCK_25_AND_20_MHZ

        K1_X100_XCONV = 2.991015224934400e+003F; /* NOTE:- These constants are clock frequency dependent */
        K2_X100_XCONV = 2.255716823859200e+002F;
        K3_X100_XCONV = 6.15898310246400F;

#endif

#ifdef CLOCK_28_5_MHZ

        K1_X100_XCONV = 2.623697565731930e+003F; /* NOTE:- These constants are clock frequency dependent */
        K2_X100_XCONV = 1.978698968297544e+002F;
        K3_X100_XCONV = 5.40261675654737F;

#endif

#ifdef CLOCK_24_MHZ

        K1_X100_XCONV = 3.115640859306666e+003F; /* NOTE:- These constants are clock frequency dependent */
        K2_X100_XCONV = 2.349705024853334e+002F;
        K3_X100_XCONV = 6.41560739840000;

#endif

    }
    else
    {   
        /* 1Hz loop */
        K1 = 0.02729F;
        K2 = 2.562e-4F;
        K3 = 8.084e-7F;

#ifdef CLOCK_25_AND_20_MHZ        

        K1_X100_XCONV = 4.688386300313599e+002F; /* NOTE:- These constants are clock frequency dependent */
        K2_X100_XCONV = 4.40148248494080F;
        K3_X100_XCONV = 0.01388820624835F;

#endif

#ifdef CLOCK_28_5_MHZ

        K1_X100_XCONV = 4.112619561678596e+002F; /* NOTE:- These constants are clock frequency dependent */
        K2_X100_XCONV = 3.86094954819368F;
        K3_X100_XCONV = 0.01218263705995F;

#endif
    
#ifdef CLOCK_24_MHZ

        K1_X100_XCONV = 4.883735729493333e+002F; /* NOTE:- These constants are clock frequency dependent */
        K2_X100_XCONV = 4.58487758848000F;
        K3_X100_XCONV = 0.01446688150869;

#endif

    }
        
    /* Second code filter integrator */
    code_filter_state_2[sub_chn][chn] += code_filter_state_1[sub_chn][chn];


    if(sys[chn] == GPS)
        TempState = STATE_5F;  /* was 5F */
    else
        TempState = STATE_7F;

    if(tracking_state[chn] <= TempState)
    {       
        /* Execute filter */
        phi_k  = (K1*phase_error + K2*code_filter_state_1[sub_chn][chn] + K3*code_filter_state_2[sub_chn][chn])/T;
        phi_k *= ONE_HZ_WORD_CODE;  /* Scale to hardware words */
    }           
    else           
    {                                      
        /********************************************************************/
        /*                                                                  */
        /* When the tracking state is greater than or equal to STATE_6 the  */
        /* observables for the receiver may be used. To produce the best    */
        /* observables the filter is re-arranged to reduce numerical        */
        /* truncation. The possible truncation is only present at high      */
        /* dynamics and so it is not necessary to perform the following     */
        /* during acquisition.                                              */
        /*                                                                  */
        /********************************************************************/
        
        integer = (long)code_filter_state_2[sub_chn][chn];
        code_filter_state_2[sub_chn][chn] -= (float)integer;
        
        phi_k = ( K1_X100_XCONV * phase_error 
                + K2_X100_XCONV * code_filter_state_1[sub_chn][chn]
                + K3_X100_XCONV * code_filter_state_2[sub_chn][chn]);
                              
        /* To avoid numerical truncation the second state is partitioned    */
        /* into an integer & float part. The float part is processed above. */
        /* The integer part is processed below. In order to prevent the     */
        /* second integrator from becoming a large number the integer part  */
        /* is reset at each epoch. In order to do this the output of the    */
        /* integer stage of the filter is accumulated.                      */
        
        code_filter_sum[sub_chn][chn] += K3_X100_XCONV * (float)integer;
    }    
                 
    /* Update code NCO frequency word                                            */
    /* New frequency consists of:-                                               */
    /*      aid_freq - estimate of Doppler from carrier Doppler (carrier aiding) */
    /*      code_freq - nominal chipping rate                                    */
    /*      phi_k     - filtered error signal - essentially to allow code carrier*/
    /*                  divergence due to the ionsophere.                        */

    /* Need to account for float to int rounding (Hence the 0.5)      */
    /*                                                                */
    /* Note:- to avoid numerical truncation the integer & decimal     */
    /*        parts of the nominal code frequency have been separated.*/

    conv = phi_k + code_filter_sum[sub_chn][chn] + decimal_code_freq + aid_freq;

    if(conv < 0.0F)
        conv -= 0.5F;
    else
        conv += 0.5F;

    code_freq_word[sub_chn][chn] = code_freq + (long)(conv);
    
    /* update the hardware with the new frequency */
    if(sub_chn == CA_primary)
    {
        Set_Code_NCO(chn,CACODE,code_freq_word[sub_chn][chn], p_myIRQ);
    }
    else
    {
        if(band == L1)
            Set_Code_NCO(chn,P1CODE,code_freq_word[sub_chn][chn], p_myIRQ);
        else
            Set_Code_NCO(chn,P2CODE,code_freq_word[sub_chn][chn], p_myIRQ);
    }
}

void steady_state_long_integration_CA_tracking(int chn, Irq *p_myIRQ)
{
    /* Read the I and Q C/A correlators */
    read_correlator_totals(chn,CA_primary,FINE);
    track_CA(chn, p_myIRQ);
            
    /***********************************************************************/
    /*                                                                     */
    /* Next section initializes the P-code generators at the correct epoch */
    /*                                                                     */
    /***********************************************************************/
            
    if(sys[chn] == GPS)
    {   
        start_GPS_p(chn, p_myIRQ);

        if(task_counter[chn] == 100)
        {
            sys_time[chn]++;
            task_counter[chn] = 0;
			// PQ: These two variables are then used in function start_GPS_p.
        }
    }
    else // PQ: it is for GLONASS, comment out 090721
    {
/*        if(task_counter[chn] == 98)
        {
            Set_Code_NCO(chn,P1CODE,0L, p_myIRQ);
        }

        if(task_counter[chn] == 99)
        {
            // Only set-up C/A ms counter the first tiume around

            Set_Flags(chn,PRIMARY,TIM1ARM, p_myIRQ);
            Clear_Flags(chn,PRIMARY,TIM1ARM, p_myIRQ);
            
            tracking_state[chn] = STATE_7;
            p_search_int_count[chn] = 0;//GB added for 98 f/w compatibility
            
            Initialise_P_Code(chn,GLONASS,0,0, p_myIRQ);
            Initialise_W_Control(chn,10,20,40,NO_W,NO_W,ACC_A+ACC_B, p_myIRQ);
            Set_Flags(chn,P1CTRL,1, p_myIRQ);  // Sets ACCMODE 
            Set_Flags(chn,P2CTRL,1, p_myIRQ);  // Sets ACCMODE 
            
            code_freq_word[P_L1][chn] = code_freq_word[CA_primary][chn] * 10UL;
            code_freq_word[P_L2][chn] = code_freq_word[CA_primary][chn] * 10UL;
            
            Set_Code_NCO(chn,P1CODE,code_freq_word[P_L1][chn], p_myIRQ);
            Set_Code_NCO(chn,P2CODE,code_freq_word[P_L2][chn], p_myIRQ);
            
            Set_Lag_Spacing(chn,PRIMARY,W_E_L+W_4T_SPACING+CA_Lag_Mode[chn]+FULL, p_myIRQ);
            Set_Lag_Spacing(chn,SECONDARY,W_E_L+W_4T_SPACING, p_myIRQ);
            Clear_Flags(chn,GLOBAL, L1_on_L2, p_myIRQ);
            
            carrier_aid_L2(chn,0,FALSE, p_myIRQ);
            
            //GB commented out if statement to make compatible with 98 f/w
            //if(re_acquire_p[chn] == FALSE)
                code_lock_thres[CA_primary][chn] = STATE_7_CA_THRES[sys[chn]] + 1L;


            // remove 511/64 chips in order to partially account for the
            // pipe delays in the P-code hardware.                   
            
            Shift_Code_NCO_Phase(chn,P1CODE,-1,ASYNC,TRUE, p_myIRQ);
            Shift_Code_NCO_Phase(chn,P2CODE,-1,ASYNC,TRUE, p_myIRQ);
            Shift_Code_NCO_Phase(chn,P1CODE,-1,ASYNC,FALSE, p_myIRQ);
            Shift_Code_NCO_Phase(chn,P2CODE,-1,ASYNC,FALSE, p_myIRQ);
            Shift_Code_NCO_Phase(chn,P1CODE,-145,SYNC,FALSE, p_myIRQ);  // was -161
            Shift_Code_NCO_Phase(chn,P2CODE,-145,SYNC,FALSE, p_myIRQ);  // was -161
            
            re_acquire_p[chn] = FALSE;
            p_chips_added[chn] = 0;
            shift_p[chn] = FIRST;
            
        }

        if( (task_counter[chn] % 100) == 0)
        {
            sys_time[chn]++;
            task_counter[chn] = 0;
        }*/
    }

    task_counter[chn]++;
    
}

void track_CA(int chn, Irq *p_myIRQ)
{
    float code_phase_error;
    float carr_phase_error;
    int sign;
    float K1_X100_XWORD; 
    float K2_X100_XWORD; 
    float K3_X100_XWORD; 
    int bit_length;
    float f_out;
    long integer;
    long carr_filter_state_2_long;
    float conv;
    
    /* Change the L1 carrier tracking to the P L1 signal when locked */
    /* INTERFERENCE WORK */
/*
    if(tracking_state[chn] >= 18)
        carr_phase_error  = (-1) * calc_carrier_error(chn,P_L1,&sign,L1,0.01F,FALSE,NINETY_DEG);
    else
        carr_phase_error  = calc_carrier_error(chn,CA_primary,&sign,L1,0.01F,FALSE,NINETY_DEG);
*/
    /* Calculate the carrier tracking error for the C/A code */
    carr_phase_error  = calc_carrier_error(chn,CA_primary,&sign,L1,0.01F,FALSE,NINETY_DEG);

    /*********************************************************************/
    /*                                                                   */
    /* Using a non-coherent dot-product calculate the code phase tracking*/
    /* error - note I(E-l) and Q(E-L) is formed in hardware.             */
    /*                                                                   */
    /*********************************************************************/

    code_phase_error = dot_product_discriminator(CA_primary,chn,FALSE);

    /* The code_phase_error is in code chips! */

    /***********************************************************************/
    /*                                                                     */
    /* Now have the code and carrier discriminator tracking error values   */
    /*                                                                     */
    /*          carr_phase_error & code_phase_error                        */
    /*                                                                     */
    /***********************************************************************/

    /* Update the 3rd order carrier sums */
    
    /* first integrator */
    carr_filter_state_1_float[L1][chn] += carr_phase_error;
    
    /* Seperate first integrator into integer and floating point components */
    /* this is to reduce numerical truncation in the integration.           */
	// integer point components:  carr_filter_state_1_long
	// floating point components: carr_filter_state_1_float
    integer = (long)carr_filter_state_1_float[L1][chn];
    carr_filter_state_1_float[L1][chn] -= (float)integer;
    carr_filter_state_1_long[L1][chn]  += integer;
    
    /* Form the second integration. Note the integer and fractional float   */
    /* summations are treated seperately to avoid numerical problems.       */
    /* The long part of state 2 is not stored as there is an external       */
    /* accumulation is performed to reduce the numerical truncation.        */
    carr_filter_state_2_long            = carr_filter_state_1_long[L1][chn];
    carr_filter_state_2_float[L1][chn] += carr_filter_state_1_float[L1][chn];

    /* Remove from the float accumulation any integer component             */
    integer = (long)carr_filter_state_2_float[L1][chn];
    carr_filter_state_2_float[L1][chn] -= (float)integer;
    carr_filter_state_2_long           += integer;

    /* Update the 2nd order code sum - first integrator */
    code_filter_state_1[CA_primary][chn] += code_phase_error;
             
    /* 10Hz carrier filter coefficients */

#ifdef CLOCK_25_AND_20_MHZ

    K1_X100_XWORD =  3.589218269921280e+004F;/* NOTE:- These constants are clock frequency dependent */
    K2_X100_XWORD =  2.706860188631040e+003F;
    K3_X100_XWORD = 73.90779722956800F;

#endif

#ifdef CLOCK_28_5_MHZ

    K1_X100_XWORD = 3.148437078878316e+004F;    /* NOTE:- These constants are clock frequency dependent */
    K2_X100_XWORD = 2.374438761957054e+003F; 
    K3_X100_XWORD = 64.83140107856843F;

#endif
    
#ifdef CLOCK_24_MHZ

    K1_X100_XWORD = 3.738769031168000e+004F;    /* NOTE:- These constants are clock frequency dependent */
    K2_X100_XWORD = 2.819646029824001e+003F;
    K3_X100_XWORD = 76.98728878080000F;

#endif

    /* Execute the "JPL" 3rd order carrier filter */
    f_out =    K1_X100_XWORD * carr_phase_error 
             + K2_X100_XWORD * carr_filter_state_1_float[L1][chn]
             + K3_X100_XWORD * carr_filter_state_2_float[L1][chn];
                      
    /* Add the comonent for the integer part of the first state */
    f_out += K2_X100_XWORD * (float)carr_filter_state_1_long[L1][chn];
    
    /* The integer part of the second integration is now calculated */
    /* As this number will get very large at high Dopplers the      */
    /* integer and floating point are seperated so that numerical   */
    /* truncation does not occur (1 + 1e-6 = 1 for floating point). */
    carr_filter_sum_float[chn] += K3_X100_XWORD * (float)carr_filter_state_2_long;
    integer                     = (long)carr_filter_sum_float[chn];
    carr_filter_sum_long[chn]  += integer;
    carr_filter_sum_float[chn] -= (float)integer;
    
    /* Update the carrier NCO word */
    conv = f_out + carr_filter_sum_float[chn];
    if(conv < 0.0F)
        conv -= 0.5F;
    else
        conv += 0.5F;
    carrier_low[L1][chn] = carr_filter_sum_long[chn]  + freq_offset[chn] + (long)(conv);
    carrier_low[L1][chn] -= freq_offset_top[chn]*TWO_POWER_32;

    /* Account for the carrier word being 36 bits */
    carrier_word_wrap(chn,L1,TRUE);

    /* update the carrier hardware with the new frequency */
    Set_Carrier_NCO(chn,L1CARRIER,carrier_top[L1][chn],carrier_low[L1][chn], p_myIRQ);

    /* Call the modified code filter (aided by the carrier) & output  */
/*
    if(sys[chn] == GPS)
    {
        code_aid_integrated(chn,code_phase_error,CA_primary,L1,0.01F);
        bit_length = 20;
    }
    else
    {
        code_aid_integrated_second(chn,code_phase_error,CA_primary);
        bit_length = 10;
    }
*/
    /* Up until track_CA is called, GLONASS C/A is tracked using a 3rd order */
    /* filter.  Here it is suddenly changed to 2nd order. (steady_state_long_integration */
    /* onward).  Have replaced this with the proven 3rd order loop.  P-code */
    /* still tracks with a 2nd order loop */

    code_aid_integrated(chn,code_phase_error,CA_primary,L1,0.01F, p_myIRQ);

    if(sys[chn] == GPS)
        bit_length = 20;
    else
        bit_length = 10;

    sign_store[chn] += I_B[CA_primary][chn];    /* Accumulate I on time */

    /* At the end of the bit make the decision */
    if(bit_counter[chn] == bit_length)
    {
        data_store[chn] <<= 1;

        /* If the correlator sum is positive then the bit is a one */
        if(sign_store[chn] > 0)
            data_store[chn] += 1;   /* save in data_store[] for transfer to the host */

        sign_store[chn] = 0;
        bit_counter[chn] = 0;
        num_valid_bits[chn]++;

        if(num_valid_bits[chn] >= 16)
        {
            /* The raw nav bits are NOT written to independant */
            /* banks, bank ignored                             */
            /*GB - don't need for IGEX so disable
            write_to_dual_port(chn,0,RAW_NAV_BITS,0);
            */
            num_valid_bits[chn] = 0;
        }
    }

    bit_counter[chn] += 10;    /* Update the bit counter by the length of the correlation period */
}

void start_GPS_p(int chn, Irq *p_myIRQ)
{                         
    /* On the interrupt before the P code generator is setup make sure */
    /* that the P code NCO is not running. This ensures that the P     */
    /* code generator is correctly initalized.                         */
    if( ((sys_time[chn] + 1)%6 == 0) && ((task_counter[chn]%100) == 98))
    {
        Set_Code_NCO(chn,P1CODE,0L, p_myIRQ);
        /*
        Set_Flags(chn,PRIMARY,TIMARM);
        Clear_Flags(chn,PRIMARY,TIMARM);
        */
    }
        
        
    /* On the epoch before the start of the line, calculate the HOW and*/
    /* set the P code generator and NCO. They will activate on the     */
    /* next epoch which is the start of the line.                      */
    
    if( ((sys_time[chn] + 1)%6 == 0) && ((task_counter[chn]%100) == 99))
    {
        /* The C/A millisecond counter needs to be set-up in the 1st run */
        /* through this function.  After this, the additional ms are     */
        /* included in the pseudorange and so do not need to be reset    */

        Set_Flags(chn,PRIMARY,TIM1ARM, p_myIRQ);
        Clear_Flags(chn,PRIMARY,TIM1ARM, p_myIRQ);
    	cio << bold << "enter STATE_7" << normal << endl; cio << noshowcursor << flush;
        tracking_state[chn] = STATE_7;  /* Start P-code acquisition next time through */
        p_search_int_count[chn] = 0;/*GB added to make compatible with 98 f/w*/
            
        /* set the P-code generator up with the HOW */
        Initialise_P_Code(chn,GPS,channel_num[chn],(sys_time[chn] + 1)/6, p_myIRQ);


        if(CODELESS_TRACKING[chn] == FALSE)
        {
            Initialise_W_Control(chn,20,20,40,NO_W,NO_W,ACC_A+ACC_B, p_myIRQ);
            /****************************************************************/
            Set_Flags(chn,P1CTRL,1, p_myIRQ);  /* Sets ACCMODE */
            Set_Flags(chn,P2CTRL,1, p_myIRQ);  /* Sets ACCMODE */
        }
        else
        {
            /* Initialise_W_Control(chn,20,20,40,NO_W,CROSS_W,ACC_A+ACC_B); */
            /* Initialise_W_Control(chn,20,20,40,CROSS_W,CROSS_W,ACC_A+ACC_B); */

            /*GB - was this but latest f/w uses the next set up*/
            /*Initialise_W_Control(chn,20,20,40,CROSS_W,CROSS_W,ACC_A+ACC_B);*/

            Initialise_W_Control(chn,22,22,40,CROSS_W,CROSS_W,ACC_A+ACC_B, p_myIRQ);
            Set_Flags(chn,P1CTRL,1, p_myIRQ);  /* Sets ACCMODE */
            Set_Flags(chn,P2CTRL,1, p_myIRQ);  /* Sets ACCMODE */

            filtered_carr_error[L2][chn] = 90.0F/360.0F; // PQ 20090711 added /360, because the unit should be cycle
        }

        /* Set the NCO frequency to a scaled version of the current C/A code frequency */
        code_freq_word[P_L1][chn] = code_freq_word[CA_primary][chn] * 10UL;
        code_freq_word[P_L2][chn] = code_freq_word[CA_primary][chn] * 10UL;
            
        /* Output the frequency to the code generator */
        Set_Code_NCO(chn,P1CODE,code_freq_word[P_L1][chn], p_myIRQ);
        Set_Code_NCO(chn,P2CODE,code_freq_word[P_L2][chn], p_myIRQ);
            
        /* Currently sets the first correlator to E-L */
        /* According to P_code_tracking, the mode of P-code correlators
           should be Punct. and Early--here it is not!!
           Changed the PRIMARY to W_EARLY for debug purposes initially
           since a) the search does NOT use early so it is not needed;
           b) E-L mode is selected again when P-code energy is detected.
           JC - 12/10/95
        */

        Set_Lag_Spacing(chn,PRIMARY,W_E_L+W_2T_SPACING+CA_Lag_Mode[chn]+FULL, p_myIRQ); /* ~ 1chip */
        Set_Lag_Spacing(chn,SECONDARY,W_E_L+W_2T_SPACING, p_myIRQ);

/*
        Set_Lag_Spacing(chn,PRIMARY,W_EARLY+W_2T_SPACING+CA_Lag_Mode[chn]+FULL); / ~ 1chip /
        Set_Lag_Spacing(chn,SECONDARY,W_EARLY+W_2T_SPACING);
*/
        /* Debug will now dump correlators before and after a switch... */

        /* Stop the correlators cascading - ie set the P code correlators to  */
        /* correlate with the P code and not the C/A code.                    */

        Clear_Flags(chn,GLOBAL, L1_on_L2, p_myIRQ);

        /* Set the L2 Doppler according to the L1 frequency */
        carrier_aid_L2(chn,0,FALSE,p_myIRQ);
            
        /*GB - taken out if statement  to make compatible with 98 f/w
        if(re_acquire_p[chn] == FALSE) */
            code_lock_thres[CA_primary][chn] = STATE_7_CA_THRES[sys[chn]] + 1L;

        /* add 511/64 chips in order to partially account for the */
        /* pipe delays in the P-code hardware.                    */
            
        Shift_Code_NCO_Phase(chn,P1CODE,1,ASYNC,TRUE, p_myIRQ);
        Shift_Code_NCO_Phase(chn,P2CODE,1,ASYNC,TRUE, p_myIRQ);
        Shift_Code_NCO_Phase(chn,P1CODE,1,ASYNC,FALSE, p_myIRQ);
        Shift_Code_NCO_Phase(chn,P2CODE,1,ASYNC,FALSE, p_myIRQ);
    
        /* Additional tests have shown that this still isn't enough phase */
        /* an additional 35/64 are added JC - 31/1/96 */
/*
        Shift_Code_NCO_Phase(chn,P1CODE,35,ASYNC,FALSE);
        Shift_Code_NCO_Phase(chn,P2CODE,35,ASYNC,FALSE);
*/
        Shift_Code_NCO_Phase(chn,P1CODE,511,SYNC,FALSE, p_myIRQ);
        Shift_Code_NCO_Phase(chn,P2CODE,511,SYNC,FALSE, p_myIRQ);
        
        re_acquire_p[chn] = FALSE;
        shift_p[chn] = FIRST;
        p_chips_added[chn] = 0;
    }

}

void carrier_aid_L2(int chn, float phase_error,int filter_flag, Irq *p_myIRQ)
{
    float phi_k;   /* filtered code phase error in degrees */
    float K1,K2;
    float L1_L2_ratio;
    float T;
    long carrier_nominal_freq;
    float aid_freq;
    long freq;
    float conv;

    switch(sys[chn])
    {
        case GLONASS:
            /* The carrier control word consists of two parts to calculate the */
            /* absolute carrier Doppler determine the nominal value of the two */
            /* words and subtract from the actual words (scaling correctly)    */
            carrier_nominal_freq  = carrier_low[L1][chn] - GLN_L1_WORD[channel_num[chn]];
            carrier_nominal_freq += (carrier_top[L1][chn] - GLN_L1_TOP[channel_num[chn]]) * TWO_POWER_32;

            if(channel_num[chn] < GLONASS_FOLDOVER_CHANNEL_L1)/*GB - make carrier_nominal_freq the valid Doppler*/
                carrier_nominal_freq *= -1;
                
            /* ratio of L1 & L2 Dopplers */
            L1_L2_ratio = 0.77777777777778F; /* 7.0F/9.0F */
            
            if(channel_num[chn] < GLONASS_FOLDOVER_CHANNEL_L2) /*GB calculate correct L2 Doppler */
               L1_L2_ratio *= -1.0F;
               
            break;
            
        case GPS:    
            /* The carrier control word consists of two parts to calculate the */
            /* absolute carrier Doppler determine the nominal value of the two */
            /* words and subtract from the actual words (scaling correctly)    */

            carrier_nominal_freq  = carrier_low[L1][chn] - GPS_L1_WORD;
            carrier_nominal_freq += (carrier_top[L1][chn] - GPS_L1_TOP) * TWO_POWER_32;

            L1_L2_ratio = 0.77922077922078F;    /* 60/77 */
            
            break;

        default:
            error_type = INVALID_SATELLITE_SYSTEM;
            return;
    } 

    aid_freq = L1_L2_ratio*(float)carrier_nominal_freq;


    if(filter_flag == TRUE)
    {   
        /* Run the filter */
        carr_filter_state_1_float[L2][chn] += phase_error;

        T = 0.01F;

        if((CODELESS_TRACKING[chn] == FALSE) && ((tracking_state[chn] < STATE_7E) || (FINAL_L2_BW == 10)))
        {
            /* Coefficients for a 10Hz bandwdith with a 10ms integration period */
            K1 = 0.1911F;
            K2 = 0.01305F;
        }
        else
        {
            /* Coefficients for a  1Hz bandwdith with a 10ms integration period */
            K1 = 0.03003F;
            K2 = 2.344e-4F;
        }

        phi_k = (K1*phase_error + K2*carr_filter_state_1_float[L2][chn])/T;

        phi_k *= ONE_HZ_WORD_CARR;  /* Scale to hardware words */


        conv = phi_k + aid_freq;
        if(conv < 0.0F)
            conv -= 0.5F;
        else
            conv += 0.5F;

        freq = (long)(conv);
    }
    else    
    {
        /* Scale the L1 frequency to L1 without running the filter - used during L2 search */
        if(aid_freq < 0.0F)
            aid_freq -= 0.5F;
        else
            aid_freq += 0.5F;
        freq = (long)(aid_freq);
    }
    
    /* Now set the hardware carrier word */
    if(sys[chn] == GLONASS)
    {    
        carrier_top[L2][chn] = GLN_L2_TOP[channel_num[chn]];
        carrier_low[L2][chn] = GLN_L2_WORD[channel_num[chn]] + freq;
    }
    else
    {
        carrier_top[L2][chn] = GPS_L2_TOP;
        carrier_low[L2][chn] = GPS_L2_WORD + freq;
    }
    
    /* Prevent out of range values */
    carrier_word_wrap_loop(chn,L2);
    
    Set_Carrier_NCO (chn,L2CARRIER,carrier_top[L2][chn],carrier_low[L2][chn], p_myIRQ);

}

void P_code_tracking(int chn, Irq *p_myIRQ)
{
    /* Read the I and Q C/A correlators */
    read_correlator_totals(chn,CA_primary,FINE);

    /* execute the C/A code tracking filters */
    track_CA(chn, p_myIRQ);
    
    /********************************************************************/
    /*                                                                  */
    /* During the P code search both E & P correlators are available to */
    /* detect energy. Once energy has been detected the E correlator    */
    /* switches to E-L. Because under steady state this number will be  */
    /* small an extra 4 bits are read from the hardware to avoid        */
    /* numerical truncation in the code tracking, hence the FINE option */
    /* in the correlator read.                                          */
    /*                                                                  */
    /********************************************************************/
    if(tracking_state[chn] == STATE_7)
    {
        read_correlator_totals(chn,P_L1,NORM);
        read_correlator_totals(chn,P_L2,NORM);
        P_code_timeout[chn]++; // PQ 090720 this variable is not currently used.
        if((shift_p[chn] == FIRST) && (p_chips_added[chn] == 0)) // PQ 090720 ??????????????????
        {
            Shift_Code_NCO_Phase(chn,P1CODE,511,ASYNC,FALSE, p_myIRQ); 
            Shift_Code_NCO_Phase(chn,P2CODE,511,ASYNC,FALSE, p_myIRQ);
        }
    }
    else
    {
        read_correlator_totals(chn,P_L1,FINE);
        read_correlator_totals(chn,P_L2,FINE);
    }

    if(    ((CODELESS_TRACKING[chn] == FALSE) && (tracking_state[chn] != STATE_7 ))
        || ((CODELESS_TRACKING[chn] == TRUE ) && (tracking_state[chn] >= STATE_7A)) )
    {
        if(CODELESS_TRACKING[chn] == TRUE) // CODELESS_TRACKING and tracking_state[chn] >= STATE_7A
        {
            p_codeless(chn, p_myIRQ);
        }
        else
        {
			// PQ: since P Code is always P(Y) Code nowadays, comment out the whole section.
			exit(1);
            /*track_P_L1(chn);
			          
			if(tracking_state[chn] < STATE_7C)
            {
                / L2 P code is not locked so continue acquisition /
                acquire_P_L2(chn);
            }
            else
            {
                / L2 P code is locked - execute the L2 P code filter /
                track_P_L2(chn);
            }

          	//GB made compatible with 98 f/w
            // If tracking L1 P with a 1Hz filter
            if(tracking_state[chn] > STATE_7A)
            {
                if(tracking_state[chn] < STATE_7C)
                {
                    // L2 P code is not locked so continue acquisition
                    acquire_P_L2( chn);
                }
                else
                {
                    // L2 P code is locked - execute the L2 P code filter
                    track_P_L2(chn);
                }
            }
			*/                    
        }
    }
    else
    {
        /* L1 P code is not locked so continue acquisition */
        if(CODELESS_TRACKING[chn] == TRUE) // CODELESS_TRACKING and tracking_state[chn] is STATE_7
            acquire_PW_codeless(chn, p_myIRQ);
        else
		{
			// PQ: since P Code is always P(Y) Code nowadays, comment out the whole section.
			exit(1);
			/*
            acquire_P_L1_coded(chn);
			*/
		}


        if((shift_p[chn] == FIRST) && (p_chips_added[chn] == 4))
        {

/*
            Shift_Code_NCO_Phase(chn,P1CODE,90,ASYNC,FALSE);
            Shift_Code_NCO_Phase(chn,P2CODE,90,ASYNC,FALSE);

*/

            Shift_Code_NCO_Phase(chn,P1CODE,60,ASYNC,FALSE, p_myIRQ);
            Shift_Code_NCO_Phase(chn,P2CODE,60,ASYNC,FALSE, p_myIRQ);

        }
    }    

    /* This section has been temporarily been re-added--if GLN operation    */
    /* it obviously doesn't work.  This whole section needs to be rethought */
    /* and rewritten as it doesn't work properly... JC 28/3/96              */

/*GB disabled to make compatible with 98 f/w*/
/*    if((tracking_state[chn] == STATE_7) && (re_acquire_p[chn] == TRUE))
    {
        start_GPS_p(chn);
    }
*/
    if( (task_counter[chn] % 100) == 0)
    {
        sys_time[chn]++;
        task_counter[chn] = 0;
    }

/* Uncommented JC 4/1/96 */

    if(CODELESS_TRACKING[chn] == FALSE)
	{
		// PQ: since P Code is always P(Y) Code nowadays, comment out the whole section.
		exit(1);
        // check_lock_p_code_operation(chn);
	}
    else    
        check_lock_p_codeless_operation(chn, p_myIRQ);

/*
    if(tracking_state[chn] == STATE_7E)
    {
        tracking_state[chn] = STATE_7F;
    }
*/
    task_counter[chn]++;
    p_search_int_count[chn]++;/*GB made compatible with 98 f/w*/
    
}

void p_codeless(int chn, Irq *p_myIRQ)
{
    float carr_phase_error = 0.0F;
    float code_phase_error = 0.0F;
    int sign;
    
    /*****************************************************************/
    /*****************************************************************/
    /**                                                             **/
    /**                      L1 Processing                          **/
    /**                                                             **/
    /*****************************************************************/
    /*****************************************************************/


/*
    if((tracking_state[chn] >= STATE_7F) && (p_search_int_count[chn]%10 == 0))
    {
        dynamic_bandwidth(chn,CNR);
    }   
*/
    
/*    if(p_search_int_count[chn] == 400)*/
    /*GB made compatible with 98 f/w */
	// PQ: this block is only available for L1 Processing
    if( (p_search_int_count[chn] == 400) && (tracking_state[chn] < STATE_7E) )
    {
        tracking_state[chn] = STATE_7E;
        cio << "enter STATE_7E from p_codeless" << endl;
        /* Initialize lock detects */
        code_lock_thres[P_L1][chn] = STATE_7_P1_THRES_CODELESS[sys[chn]]+1L;
        code_lock_thres[P_L2][chn] = STATE_7_P2_THRES_CODELESS[sys[chn]]+1L;
    }    

    /* Reintroduced the 0.1Hz filters for codeless 2/4/96 */
/*
    if(p_search_int_count[chn] == 1000)
    {
                                  
        change_codeless_bandwidth(ONE_POINT_0,ZERO_POINT_1,chn,P_L1,L1,CODE);
        change_codeless_bandwidth(ONE_POINT_0,ZERO_POINT_1,chn,P_L2,L2,CODE);
        change_codeless_bandwidth(ONE_POINT_0,ZERO_POINT_1,chn,P_L2,L2,CARR);
        codeless_bandwidth[chn] = ZERO_POINT_1;
    }
*/
                                             
    /* The hardware integrates over 10ms - must further integrate in */
    /* to produce 100ms accumulations of I & Q correlation totals    */

    /* Increase the integration period to 200ms JC 3/4/96 */

    I_A_INT[L1][chn] += I_A[P_L1][chn];
    I_B_INT[L1][chn] += I_B[P_L1][chn];
        
    Q_A_INT[L1][chn] += Q_A[P_L1][chn];
    Q_B_INT[L1][chn] += Q_B[P_L1][chn];

            
    if((p_search_int_count[chn]%20) == 0)   /* Increased from 10 for 200ms int */
    {
        /* Form totals to send to PC as correlators */
        I_A_PC[L1][chn] = I_A_INT[L1][chn];
        I_B_PC[L1][chn] = I_B_INT[L1][chn];
        Q_A_PC[L1][chn] = Q_A_INT[L1][chn];
        Q_B_PC[L1][chn] = Q_B_INT[L1][chn];


        /* Execute the filter once every 100ms */
        /* Now every 200ms */

        /* Calculate the code phase error */
        code_phase_error = dot_product_discriminator(P_L1,chn,TRUE);

        /* Update the 2nd order  code sum */
        code_filter_state_1[P_L1][chn] += code_phase_error;

        /* Execute the L1 code filter (aided by L1 carrier) */

        code_aid_codeless(chn,code_phase_error,P_L1,L1,codeless_bandwidth[chn],TRUE, p_myIRQ);

        I_A_INT[L1][chn] = 0L;
        I_B_INT[L1][chn] = 0L;
        Q_A_INT[L1][chn] = 0L;
        Q_B_INT[L1][chn] = 0L;
    }
    else
    {   
        /* The code output is updated every 10ms with an aiding signal from  */
        /* the C/A code - this prevents large errors and allows high dynamic */
        /* tracking.                                                         */
        
        code_aid_codeless(chn,code_phase_error,P_L1,L1,codeless_bandwidth[chn],FALSE, p_myIRQ);
    }
    
    /*****************************************************************/
    /*****************************************************************/
    /**                                                             **/
    /**                      L2 Processing                          **/
    /**                                                             **/
    /*****************************************************************/
    /*****************************************************************/
    
    I_A_INT[L2][chn] += I_A[P_L2][chn];
    I_B_INT[L2][chn] += I_B[P_L2][chn];
        
    Q_A_INT[L2][chn] += Q_A[P_L2][chn];
    Q_B_INT[L2][chn] += Q_B[P_L2][chn];

        
    if(p_search_int_count[chn]%20 == 0)  /* Increased from 10 for 200ms */
    {
        /* Form totals to send to PC as correlators */
        I_A_PC[L2][chn] = I_A_INT[L2][chn];
        I_B_PC[L2][chn] = I_B_INT[L2][chn];
        Q_A_PC[L2][chn] = Q_A_INT[L2][chn];
        Q_B_PC[L2][chn] = Q_B_INT[L2][chn];


        /* Integrate for a further 100ms */
        /* Now 200ms */
		// PQ: this block is only available for L2 Processing
		//
        if(tracking_state[chn] >= STATE_7E)
        {
            carr_phase_error = calc_carrier_error(chn,P_L2,&sign,L2,0.01F,TRUE,ONE_EIGHTY_DEG);
        }
        else    
            carr_phase_error = calc_carrier_error(chn,P_L2,&sign,L2,0.01F,TRUE,NINETY_DEG);
            
        /* L2 codeless carrier tracking filter */

        carrier_aid_codeless(chn,carr_phase_error,codeless_bandwidth[chn],TRUE, p_myIRQ);
		//
		// PQ

        /* Calculate the code phase error */
        code_phase_error = dot_product_discriminator(P_L2,chn,TRUE);

        /* Update the 2nd order  code sum */
        code_filter_state_1[P_L2][chn] += code_phase_error;

        /* Execute the L2 code filter (aided by L1 carrier) */

        code_aid_codeless(chn,code_phase_error,P_L2,L2,codeless_bandwidth[chn],TRUE, p_myIRQ);

        I_A_INT[L2][chn] = 0L;
        I_B_INT[L2][chn] = 0L;
        Q_A_INT[L2][chn] = 0L;
        Q_B_INT[L2][chn] = 0L;
    }
    else
    {   
        /* The code & carrier outputs are updated every 10ms with an aiding  */
        /* signal from the C/A code - this prevents large errors and allows  */
        /* high dynamic tracking.                                            */

        carrier_aid_codeless(chn,carr_phase_error,codeless_bandwidth[chn],FALSE, p_myIRQ);// PQ: this line is only available for L2 Processing
        code_aid_codeless(chn,code_phase_error,P_L2,L2,codeless_bandwidth[chn],FALSE, p_myIRQ);
    }
                                 
    /* Update the 10ms counter */
    /*p_search_int_count[chn]++; GB made compatible with 98 f/w*/
}

void acquire_PW_codeless(int chn, Irq *p_myIRQ)
{
    long denom;

    code_freq_word[P_L1][chn] = code_freq_word[CA_primary][chn] * 10UL;
    code_freq_word[P_L2][chn] = code_freq_word[CA_primary][chn] * 10UL;
    Set_Code_NCO(chn,P1CODE,code_freq_word[P_L1][chn], p_myIRQ);
    Set_Code_NCO(chn,P2CODE,code_freq_word[P_L2][chn], p_myIRQ);
    
    /* Set the L2 carrier frequency from L1  - do not run the filter */
    carrier_aid_L2(chn,0,FALSE, p_myIRQ);

    I_B_INT[L1][chn] +=  I_B[P_L1][chn];
    Q_B_INT[L1][chn] +=  Q_B[P_L1][chn];

    I_B_INT[L2][chn] +=  I_B[P_L2][chn];
    Q_B_INT[L2][chn] +=  Q_B[P_L2][chn];

    /*p_search_int_count[chn]++; GB made compatible with 98 f/w*/
    
    if(p_search_int_count[chn]%10 == 0)
    {
        denom  = I_B_INT[L1][chn]*I_B_INT[L1][chn];
        denom += Q_B_INT[L1][chn]*Q_B_INT[L1][chn];

        if(p_search_int_count[chn] > 10L)
        {
            if(denom > (500L*SIGMA_2[sys[chn]]))  /* Tested at 300 JC 2/4/96 */
            {
                if(sys[chn] == GLONASS)
                {
                    Set_Lag_Spacing(chn,PRIMARY,W_E_L+W_4T_SPACING+CA_Lag_Mode[chn]+FULL, p_myIRQ);
                    Set_Lag_Spacing(chn,SECONDARY,W_E_L+W_4T_SPACING, p_myIRQ);
                }
                else
                {
                    Set_Lag_Spacing(chn,PRIMARY,W_E_L+W_2T_SPACING+CA_Lag_Mode[chn]+FULL, p_myIRQ);
                    Set_Lag_Spacing(chn,SECONDARY,W_E_L+W_2T_SPACING, p_myIRQ);
                }
                tracking_state[chn] = STATE_7A;
				cio << "enter STATE_7A from acquire_PW_codeless" << endl;
                /* Need to check when it enters the filter !!! */
                p_search_int_count[chn] = 0;
            }
            else
            {
                /* Uncommented - JC 1/11/95 */
                search_p_code(chn,TRUE, p_myIRQ);

            }
        }

        I_B_INT[L1][chn] = 0;
        Q_B_INT[L1][chn] = 0;
        I_B_INT[L2][chn] = 0;
        Q_B_INT[L2][chn] = 0;
    }

    p_chips_added[chn]+=4;
    
}

void check_lock_p_codeless_operation(int chn, Irq *p_myIRQ)
{
    /* Check for the L1 carrier unlocking */

    /* INTERFERENCE WORK - Removed to allow P code tracking to continue - JC 24/11/97 */
/* comment this out to disable CA code unlock checking */
	// PQ : this is removed according to the above comments
    //if(check_CA_lock(chn) == TRUE)
    //{
    //    return;
    //}
    //else
    //{
/* end comment */    
        /* Satellite is still locked - update the start frequency in case the */
        /* receiver loses the satellite (a new search will use this value as  */
        /* the a priori estimate).                                            */

        start_carr_freq[chn] = carrier_freq_word[L1][chn];
        carrier_top_start[chn] = carrier_top[L1][chn];
        start_code_freq[chn] = code_freq_word[CA_primary][chn];
/* comment this out to disable CA code unlock checking */
    //}
/* end comment */

    /* Check for the L2 carrier locking and then check to make sure it */
    /* remains in lock.                                                */
    if(tracking_state[chn] < STATE_7F) // STATE_7F is the final stage
    {
        if(tracking_state[chn] == STATE_7E)
        {
            if(filtered_carr_error[L2][chn] < PHASE_LOCK_THRES_CODELESS)
            {
                /* L2 carrier has locked */
                tracking_state[chn] = STATE_7F;
                cio << "STATE_7F, L2 carrier has locked" << endl;
                if(p_search_int_count[chn] > 32000)/*GB made compatible with 98 f/w */
                    p_search_int_count[chn] = 0;
            }
        }
    }
    else //  STATE_7F, the final stage
    {
        /* L2 carrier has locked - check for unlock */
        if(filtered_carr_error[L2][chn] > PHASE_LOCK_THRES_HYST_CODELESS)
        {
            /* L2 carrier has unlocked.  Reacquire L1 and L2 codeless */

            unlock[1] = TRUE;
/*
            carr_filter_state_1_float[L2][chn] = 0.0F;
            code_filter_state_1[P_L2][chn] = 0.0F;

            / Reset the carrier lock variable /
            filtered_carr_error[L2][chn] = 90.0F/360.0F; // PQ 20090711 added /360, because the unit should be cycle

            / Drop back to L2 acquisition /
            tracking_state[chn] = STATE_7;
            track_L2_time[chn] = 0;
*/
            reinitialize_p_codeless(chn, p_myIRQ);
			cio << "Phi 6" << normal << endl; cio << noshowcursor << flush;
			// PQ 090720 go back to STATE_6 from STATE_7F
	    	tracking_state[chn] = STATE_6;/*GB changed from STATE_7 to STATE_6 to make compatible with 98 f/w*/
        }
    }


    if((task_counter[chn]%100) == 0)
    {
        /* Check for code lock of P(1) & P(2) every second  (task counter updated at 0.01Hz */

        if((tracking_state[chn] == STATE_7E) || (tracking_state[chn] == STATE_7F))
        {
            /* C/A, P(1) & P(2) locked */

            if(    (code_lock_thres[P_L2][chn] < STATE_7_P2_THRES_CODELESS[sys[chn]])
                || (code_lock_thres[P_L1][chn] < STATE_7_P1_THRES_CODELESS[sys[chn]]) )
            {
                /* The L2 P-code has unlocked re-acquire */

                unlock[2] = TRUE;
/*
                carr_filter_state_1_float[L2][chn] = 0.0F;
                code_filter_state_1[P_L1][chn] = 0.0F;
                code_filter_state_1[P_L2][chn] = 0.0F;
                tracking_state[chn] = STATE_7;
                P_code_timeout[chn] = 0;
                track_L2_time[chn] = 0;
                track_L1_time[chn] = 0;
                filtered_carr_error[L2][chn] = 90.0F/360.0F; // PQ 20090711 added /360, because the unit should be cycle
                p_filter_av[L1][chn] = 0.0F;
                p_filter_av[L2][chn] = 0.0F;
                re_acquire_p[chn] = TRUE;
                p_chips_added[chn] = 0;
                shift_p[chn] = FIRST;
*/
                reinitialize_p_codeless(chn, p_myIRQ);
				cio << "Code 6" << normal << endl; cio << noshowcursor << flush;
				tracking_state[chn] = STATE_6;/*GB changed from STATE_7 to STATE_6 to make compatible with 98 f/w*/
            }
        }
        code_lock_thres[P_L2][chn] = 0L;
        code_lock_thres[P_L1][chn] = 0L;

    }        
    
    if((task_counter[chn]%20) == 0)
    {    
        /* Check C/A code every 200ms  */
        if(code_lock_thres[CA_primary][chn] < STATE_7_CA_THRES[sys[chn]])
        {
            /* C/A has unlocked - everything is driven from */
            /* this loop therefore revert to a complete     */
            /* re-acquisition. However, the last "good"     */
            /* value of Doppler can be used to reduce the   */
            /* time required to re-acquire.                 */

            /* C/A unlock disabled so P code can continue - JC 24/11/97 */
            /*GB - re-enabled C/A unlock as this was probably commented out as part of JC INTERFERENCE work*/
            
            unlock[4] = TRUE;
            code_filter_state_1[P_L1][chn] = 0.0F;
            tracking_state[chn] = STATE_7;
            P_code_timeout[chn] = 0;
            track_L2_time[chn] = 0;
            track_L1_time[chn] = 0;
            carr_filter_state_1_float[L2][chn] = 0.0F;
            code_filter_state_1[P_L2][chn] = 0.0F;
            filtered_carr_error[L2][chn] = 90.0F/360.0F; // PQ 20090711 added /360, because the unit should be cycle
            p_filter_av[L1][chn] = 0.0F;
            p_filter_av[L2][chn] = 0.0F;
            Change_Dwell_Timing (chn,PRIMARY,ONE_MS,sys[chn], p_myIRQ);     /* Change to 1ms */
            return_to_one(chn, p_myIRQ);

        }
        code_lock_thres[CA_primary][chn] = 0L;
    }

    /* Check for the P-code taking too long to acquire */
/*    if(P_code_timeout[chn] == 1000)*/
    if((p_search_int_count[chn] > 1000) && (tracking_state[chn] == STATE_7))/*GB made compatible with 98 f/w*/
    {
        /* P code lock has taken too long drop back to time */
        /* extraction in case an error has occured          */
        p_search_int_count[chn] = 0;/*GB made compatible with 98 f/w*/
/*
        P_code_timeout[chn] = 0;
        task_counter[chn] = CHANGE_TO_POINT_ZERO_FIVE_GLONASS_10ms;  / This will need to be changed for GLN /
        tracking_state[chn] = STATE_5D;
*/
    }
}

void code_aid_codeless(int chn,float phase_error,int sub_chn,int band,int BW,int filter_flag, Irq *p_myIRQ)
{   
    long carrier_nominal_freq; /* Difference between the nominal IF frequency and */
                               /* the actual IF output ie the DOPPLER             */

    float decimal_code_freq;
    unsigned long code_freq;            /* code chipping frequency */
    
    float code_carr_ratio;    /* Ratio between carrier RF freq. and PRN chipping freq. */
    
    float phi_k;   /* filtered code phase error in degrees */

    float K1,K2;

    float aid_freq;

    float conv;

    /*************************************************************************/
    /*                                                                       */
    /* Need to scale the carrier Doppler by the ratio of the code chipping   */
    /* frequency and the carrier frequency. In order to do this the nominal  */
    /* IF is subtracted from the carrier NCO output frequency (note          */
    /* calculation is in units of 'NCO words'). This provides the carrier    */
    /* Doppler. The ratio between the chipping rate and the RF frequency is  */
    /* calculated and then the estimate of code Doppler 'aid-frequency' is   */
    /* calculated.                                                           */
    /*                                                                       */
    /*************************************************************************/


    switch(sys[chn])
    {
        case GLONASS:
            /* The carrier control word consists of two parts to calculate the */
            /* absolute carrier Doppler determine the nominal value of the two */
            /* words and subtract from the actual words (scaling correctly)    */

            carrier_nominal_freq  = carrier_freq_word[L1][chn] - GLN_L1_WORD[channel_num[chn]];
            carrier_nominal_freq += (carrier_top[L1][chn] - GLN_L1_TOP[channel_num[chn]]) * TWO_POWER_32;

            /* Each GLONASS satellite transmits at a different carrier freq so */
            /* to calculate the code/carrier ratio. This is then multiplied by */
            /* the code factor of 12 because of hardware differences between   */
            /* the code and carrier NCO                                        */
            /* ie to represent a frequency of 1Hz on the carrier NCO a word    */
            /* 12 times greater than is necessary on the code NCO to produce   */
            /* the same frequency                                              */

            /* P code the code has 10 times the dynamics of the C/A */
            code_carr_ratio = GLONASS_L1_P_carr_code_ratio[channel_num[chn] - 1];
                    
            /* To account for tracking the image as the signal has folded at baseband*/
            if(channel_num[chn] < 12)
                code_carr_ratio *= -1.0;
                    
            /* code frequency word assuming no Doppler */
            code_freq = GLONASS_P_NOMINAL_CODE_WORD_INT;
            decimal_code_freq = GLONASS_P_NOMINAL_CODE_WORD_FLOAT;

            break;
            
        case GPS:    
            /* The carrier control word consists of two parts to calculate the */
            /* absolute carrier Doppler determine the nominal value of the two */
            /* words and subtract from the actual words (scaling correctly)    */

            carrier_nominal_freq  = carrier_freq_word[L1][chn] - GPS_L1_WORD;
            carrier_nominal_freq += (carrier_top[L1][chn] - GPS_L1_TOP) * TWO_POWER_32;

            /* GPS carrier freq is 1540 times the code factor of 12 because of */
            /* hardware differences between the code and carrier NCO           */
            /* ie to represent a frequency of 1Hz on the carrier NCO a word    */
            /* 12 times greater than is necessary on the code NCO to produce   */
            /* the same frequency                                              */

            /* P code the code has 10 times the dynamics of the C/A */
            code_carr_ratio = 1.0F/(154.0F*12.0F);
                
            /* GPS code frequency word assuming no Doppler */

            code_freq = GPS_P_NOMINAL_CODE_WORD_INT;
            decimal_code_freq = GPS_P_NOMINAL_CODE_WORD_FLOAT;

            break;
    }

    /* Calculate the estimate of code Doppler from the carrier Doppler */
    aid_freq = ((float)carrier_nominal_freq) * code_carr_ratio;

    if(filter_flag == TRUE)
    {
        get_filter_coeff(&K1,&K2,BW);

        phi_k = (K1*phase_error + K2*code_filter_state_1[sub_chn][chn])*10.0F;    /* 1/T = 10 (T = 100ms) */

        phi_k *= ONE_HZ_WORD_CODE;  /* Scale to hardware words */


        /* Update code NCO frequency word                                            */
        /* New frequency consists of:-                                               */
        /*      aid_freq - estimate of Doppler from carrier Doppler (carrier aiding) */
        /*      code_freq - nominal chipping rate                                    */
        /*      phi_k     - filtered error signal - essentially to allow code carrier*/
        /*                  divergence due to the ionsophere.                        */

        /* Need to account for float to int rounding (Hence the 0.5)      */
        /*                                                                */
        /* Note:- to avoid numerical truncation the integer & decimal     */
        /*        parts of the nominal code frequency have been separated.*/

        conv = phi_k + decimal_code_freq + aid_freq;
        if(conv < 0.0F)
            conv -= 0.5F;
        else
            conv += 0.5F;
        code_freq_word[sub_chn][chn] = code_freq + (long)(conv);
        codeless_code_aid_freq[band][chn] = phi_k;

    }
    else
    {
        conv = codeless_code_aid_freq[band][chn] + decimal_code_freq + aid_freq;
        if(conv < 0.0F)
            conv -= 0.5F;
        else
            conv += 0.5F;

        code_freq_word[sub_chn][chn] = code_freq + (long)(conv);
    }

    /* update the hardware with the new frequency */
    if(band == L1)
        Set_Code_NCO(chn,P1CODE,code_freq_word[sub_chn][chn], p_myIRQ);
    else
        Set_Code_NCO(chn,P2CODE,code_freq_word[sub_chn][chn], p_myIRQ);

}

void carrier_aid_codeless(int chn, float phase_error,int BW,int int_filter_flag, Irq *p_myIRQ)
{
    float phi_k;   /* filtered code phase error in degrees */
    float K1,K2;
    float L1_L2_ratio;
    long carrier_nominal_freq;
    long freq;
    float aid_freq;
    float conv;

    switch(sys[chn])
    {
        case GLONASS:
            /* The carrier control word consists of two parts to calculate the */
            /* absolute carrier Doppler determine the nominal value of the two */
            /* words and subtract from the actual words (scaling correctly)    */
            
            carrier_nominal_freq  = carrier_freq_word[L1][chn] - GLN_L1_WORD[channel_num[chn]];
            carrier_nominal_freq += (carrier_top[L1][chn] - GLN_L1_TOP[channel_num[chn]]) * TWO_POWER_32;

            if(channel_num[chn] < GLONASS_FOLDOVER_CHANNEL_L1)/*GB - make carrier_nominal_freq the valid Doppler*/
                carrier_nominal_freq *= -1;

            /* ratio of L1 & L2 Dopplers */
            L1_L2_ratio = 0.77777777777778F; /* 7.0F/9.0F */

            if(channel_num[chn] < GLONASS_FOLDOVER_CHANNEL_L2) /*GB calculate correct L2 Doppler */
               L1_L2_ratio *= -1.0F;

            break;
            
        case GPS:    
            /* The carrier control word consists of two parts to calculate the */
            /* absolute carrier Doppler determine the nominal value of the two */
            /* words and subtract from the actual words (scaling correctly)    */

            carrier_nominal_freq  = carrier_freq_word[L1][chn] - GPS_L1_WORD;
            carrier_nominal_freq += (carrier_top[L1][chn] - GPS_L1_TOP) * TWO_POWER_32;

            L1_L2_ratio = 0.77922077922078F;    /* 60/77 */
            
            break;

        default:
            error_type = INVALID_SATELLITE_SYSTEM;
            return;
    } 

    aid_freq = L1_L2_ratio*(float)carrier_nominal_freq;

    if(int_filter_flag == TRUE)
    {
        /* Run the filter */
        carr_filter_state_1_float[L2][chn] += phase_error;

        /* Filter coefficients are determined by the product T*BW */
        get_filter_coeff(&K1,&K2,BW);

        phi_k = (K1*phase_error + K2*carr_filter_state_1_float[L2][chn])*10.0F;    /* 1/T = 10 (T = 100ms) */

        phi_k *= ONE_HZ_WORD_CARR;  /* Scale to hardware words */

        conv = phi_k + aid_freq;
        
        codeless_carrier_aid_freq[chn] = phi_k;
    }
    else
    {
        conv = codeless_carrier_aid_freq[chn] + aid_freq;
    }
        
    /* Now sort out the float to long rounding error */        
    if(conv < 0.0F)
        conv -= 0.5F;
    else
        conv += 0.5F;
        
    freq = (long)(conv);

    /* Now set the hardware carrier word */
    if(sys[chn] == GLONASS)
    {    
        carrier_top[L2][chn] = GLN_L2_TOP[channel_num[chn]];
        carrier_freq_word[L2][chn] = GLN_L2_WORD[channel_num[chn]] + freq;
    }
    else
    {
        carrier_top[L2][chn] = GPS_L2_TOP;
        carrier_freq_word[L2][chn] = GPS_L2_WORD + freq;
    }    
    
    /* Prevent out of range values */
    carrier_word_wrap_loop(chn,L2);
    
    Set_Carrier_NCO (chn,L2CARRIER,carrier_top[L2][chn],carrier_freq_word[L2][chn], p_myIRQ);
}

void search_p_code(int chn,int flag, Irq *p_myIRQ)
{
    /*testjc++; GB - not used and also could overflow*/

    if(flag == FALSE)
    {
        if(shift_p[chn] == TRUE || shift_p[chn] == FIRST)
        {
            if(shift_p[chn] == FIRST)
            {
                /* First search forward for half the range */
                Shift_Code_NCO_Phase (chn,P1CODE,-4,ASYNC,FALSE, p_myIRQ);  /* Add 1/16th of a chip */
                Shift_Code_NCO_Phase (chn,P2CODE,-4,ASYNC,FALSE, p_myIRQ);  /* Add 1/16th of a chip */
                p_chips_added[chn] += 4L;
                if(p_chips_added[chn] >= (32L*RANGE[sys[chn]]))
                {
                    shift_p[chn] = FALSE;
                    p_chips_added[chn] = 0;
                }
            }
            else
            {
                Shift_Code_NCO_Phase (chn,P1CODE,-4,ASYNC,FALSE, p_myIRQ);  /* Add 1/16th of a chip */
                Shift_Code_NCO_Phase (chn,P2CODE,-4,ASYNC,FALSE, p_myIRQ);  /* Add 1/16th of a chip */
                p_chips_added[chn] += 4L;
                if(p_chips_added[chn] >= (64L*RANGE[sys[chn]]))
                {
                    shift_p[chn] = FALSE;
                    p_chips_added[chn] = 0;
                }
            }
        }
        else
        {
            Shift_Code_NCO_Phase (chn,P1CODE,4,ASYNC,FALSE, p_myIRQ);    /* Subtract 1/16th of a chip */
            Shift_Code_NCO_Phase (chn,P2CODE,4,ASYNC,FALSE, p_myIRQ);    /* Subtract 1/16th of a chip */
            p_chips_added[chn] -= 4L;
            if(p_chips_added[chn] <= -(64L*RANGE[sys[chn]]))
            {
                shift_p[chn] = TRUE;
                p_chips_added[chn] = 0;
            }
        }
    }
    else
    {
        if(shift_p[chn] == TRUE || shift_p[chn] == FIRST)
        {
            if(shift_p[chn] == FIRST)
            {
                /* First search forward for half the range */
                Shift_Code_NCO_Phase (chn,P1CODE,-1,ASYNC,FALSE, p_myIRQ);  /* Add 1/16th of a chip */
                Shift_Code_NCO_Phase (chn,P2CODE,-1,ASYNC,FALSE, p_myIRQ);  /* Add 1/16th of a chip */
                p_chips_added[chn] += 1L;
                if(p_chips_added[chn] >= (32L*RANGE[sys[chn]]))
                {
                    shift_p[chn] = FALSE;
                    p_chips_added[chn] = 0;
                }
            }
            else
            {
                Shift_Code_NCO_Phase (chn,P1CODE,-1,ASYNC,FALSE, p_myIRQ);  /* Add 1/16th of a chip */
                Shift_Code_NCO_Phase (chn,P2CODE,-1,ASYNC,FALSE, p_myIRQ);  /* Add 1/16th of a chip */
                p_chips_added[chn] += 1L;
                if(p_chips_added[chn] >= (64L*RANGE[sys[chn]]))
                {
                    shift_p[chn] = FALSE;
                    p_chips_added[chn] = 0;
                }
            }
        }
        else
        {
            Shift_Code_NCO_Phase (chn,P1CODE,1,ASYNC,FALSE, p_myIRQ);    /* Subtract 1/16th of a chip */
            Shift_Code_NCO_Phase (chn,P2CODE,1,ASYNC,FALSE, p_myIRQ);    /* Subtract 1/16th of a chip */
            p_chips_added[chn] -= 1L;
            if(p_chips_added[chn] <= -(64L*RANGE[sys[chn]]))
            {
                shift_p[chn] = TRUE;
                p_chips_added[chn] = 0;
            }
        }
    }
}

void reinitialize_p_codeless(int i, Irq *p_myIRQ)
{
    track_L2_time[i] = 0;
    track_L1_time[i] = 0;
    P_code_timeout[i] = 0;
    p_filter_av[L1][i] = 0.0F;
    p_filter_av[L2][i] = 0.0F;

    carr_filter_state_1_float[L2][i] = 0.0F;
    carr_filter_state_2_float[L2][i] = 0.0F;
    
    carr_filter_state_1_long[L2][i] = 0L;
/*
    code_filter_sum[P_L1][i] = 0.0F;
    code_filter_sum[P_L2][i] = 0.0F;
*/
    code_filter_state_1[P_L1][i] = 0.0F;
    code_filter_state_1[P_L2][i] = 0.0F;
    code_filter_state_2[P_L1][i] = 0.0F;
    code_filter_state_2[P_L2][i] = 0.0F;
                  
    filtered_carr_error[L2][i] = 90.0F/360.0F; // PQ 20090711 added /360, because the unit should be cycle

    shift_p[i] = FIRST;
    p_chips_added[i] = 0;

    I_A_INT[L1][i] = 0L;
    I_B_INT[L1][i] = 0L;
    Q_A_INT[L1][i] = 0L;
    Q_B_INT[L1][i] = 0L;

    I_A_INT[L2][i] = 0L;
    I_B_INT[L2][i] = 0L;
    Q_A_INT[L2][i] = 0L;
    Q_B_INT[L2][i] = 0L;

    codeless_code_aid_freq[L1][i] = 0.0F;
    codeless_code_aid_freq[L2][i] = 0.0F;

    codeless_carrier_aid_freq[i] = 0.0F;

    p_search_int_count[i] = 0;

    re_acquire_p[i] = TRUE;

    Set_Cycle_Difference(i,0, p_myIRQ);
}

void get_filter_coeff(float *K1,float *K2,int BW)
{
    /* The loop delay will be 10ms which is much less than */
    /* the integration time of 100ms. Therefore use the    */
    /* "No computation delay" coefficents.                 */

    switch(BW)
    {
        case ZERO_POINT_05: /* 100ms integration 0.05Hz B/W */
            *K1 = 0.01575F;
            *K2 = 6.275e-5F;
            break;

        case ZERO_POINT_1: /* 100ms integration 0.1Hz B/W */
            *K1 = 0.03109F;
            *K2 = 2.475e-4F;
            break;

        case ZERO_POINT_2: /* 100ms integration 0.2Hz B/w */
            *K1 = 0.06054F;
            *K2 = 9.607e-4F;
            break;

        case ZERO_POINT_4: /* 100ms integration 0.4Hz B/w */
            *K1 = 0.1148F;
            *K2 = 0.003619F;
            break;

        case ZERO_POINT_6: /* 100ms integration 0.6Hz B/w */
            *K1 = 0.1634F;
            *K2 = 0.007683F;
            break;

        case ZERO_POINT_8: /* 100ms integration 0.8Hz B/w */
            *K1 = 0.2070F;
            *K2 = 0.01291F;
            break;
/*
        case ONE_POINT_0: / 100ms integration 1Hz B/w /
            *K1 = 0.2461F;
            *K2 = 0.01911F;
            break;
*/

        /* Changed the integration period to 200ms which means filter */
        /* coeffs change also                                         */

        case ONE_POINT_0: /* 200ms integration 1Hz B/W */
            *K1 = 0.3864F;
            *K2 = 0.05992F;
            break;

    }
}

void my_OPfunc(char *OPstring, Irq *p_myIRQ)
{
    cio << bold << OPstring << normal << endl; cio << noshowcursor << flush;
}