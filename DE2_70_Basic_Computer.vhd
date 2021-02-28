library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
--test
entity DE2_70_Basic_Computer is
port (
	-- Inputs
	CLOCK_50			: in std_logic;
	KEY				  	: in std_logic_vector (3 downto 0);
	SW				   	: in std_logic_vector (17 downto 0);

	-- GPIO
	GPIO_0				: inout std_logic_vector (31 downto 0);
	GPIO_CLKIN_N0 	    : in std_logic;
	GPIO_CLKOUT_N0      : inout std_logic;
	GPIO_CLKIN_P0       : in std_logic;
	GPIO_CLKOUT_P0      : inout std_logic;
	
	GPIO_1				: inout std_logic_vector (31 downto 0);
	GPIO_CLKIN_N1 	    : in std_logic;
	GPIO_CLKOUT_N1      : out std_logic;
	GPIO_CLKIN_P1       : in std_logic;
	GPIO_CLKOUT_P1      : inout std_logic;

	-- Outputs
	LEDG				: out std_logic_vector (8 downto 0);
	LEDR				: out std_logic_vector (17 downto 0);

	HEX0				: out std_logic_vector (7 downto 0);
	HEX1				: out std_logic_vector (7 downto 0);

	--  Memory (SRAM)
	SRAM_ADDR			: out std_logic_vector (18 downto 0);
	SRAM_ADSC_N			: out std_logic; 						--SRAM Controller Address Status
	SRAM_ADSP_N			: out std_logic;						--SRAM Processor Address Status
	SRAM_ADV_N			: out std_logic;						--SRAM Burst Address Advance
	SRAM_BE_N			: out std_logic_vector (3 downto 0);	--SRAM Byte Write Enable
	SRAM_CE1_N			: out std_logic;						--SRAM Chip Enable 1
	SRAM_CE2			: out std_logic;						--SRAM Chip Enable 2
	SRAM_CE3_N			: out std_logic;						--SRAM Chip Enable 3
	SRAM_CLK			: out std_logic;						--SRAM Clock
	SRAM_GW_N			: out std_logic;						--SRAM Global Write Enable
	SRAM_OE_N			: out std_logic;						--SRAM Output Enable
	SRAM_WE_N			: out std_logic;						--SRAM Write Enable
	SRAM_DQ				: inout std_logic_vector (31 downto 0); --SRAM Data
	SRAM_DPA			: inout std_logic_vector (3 downto 0);  --SRAM Parity Data
	
	-- Memory (SDRAM)
	DRAM0_ADDR			: out std_logic_vector (12 downto 0);
	DRAM0_BA_1			: buffer std_logic;
	DRAM0_BA_0			: buffer std_logic;
	DRAM0_CAS_N			: out std_logic;
	DRAM0_RAS_N			: out std_logic;
	DRAM0_CLK			: out std_logic;
	DRAM0_CKE			: out std_logic;
	DRAM0_CS_N			: out std_logic;
	DRAM0_WE_N			: out std_logic;
	DRAM0_UDQM			: buffer std_logic;
	DRAM0_LDQM			: buffer std_logic;
	
	DRAM_DQ				: inout std_logic_vector (31 downto 0);
	
	DRAM1_ADDR			: out std_logic_vector (12 downto 0);
	DRAM1_BA_1			: buffer std_logic;
	DRAM1_BA_0			: buffer std_logic;
	DRAM1_CAS_N			: out std_logic;
	DRAM1_RAS_N			: out std_logic;
	DRAM1_CLK			: out std_logic;
	DRAM1_CKE			: out std_logic;
	DRAM1_CS_N			: out std_logic;
	DRAM1_WE_N			: out std_logic;
	DRAM1_UDQM			: buffer std_logic;
	DRAM1_LDQM			: buffer std_logic
	);
end DE2_70_Basic_Computer;


architecture RTL of DE2_70_Basic_Computer is

signal 			RESET		: std_logic;	
signal 			Clock_System: std_logic;								-- Clock of the system uses 96MHz Clock from Qsys
signal			clk_25		: std_logic;								-- Clock of the LCD from the Clock Bridge in Qsys
signal			clk_96		: std_logic;								--	Clock of the Camera from the Clock Bridge in Qsys


--------------------------------Signals to Recieve Camera's Pins Through GPIO_1----------------
signal 			D5M_PIXCLK 	: std_logic;								--Pixel Clock (Output) == 1 XCLKIN
signal			D5M_XCLKIN 	: std_logic;								--External Input Clock (96Mhz)	
signal         D5M_STROBE 	: std_logic;								-- Snapchot Strobe (Output)
signal         D5M_TRIGGER  : std_logic;								-- Snapshot Trigger (Input)
signal         D5M_FVAL 	: std_logic;								-- Frame Valid
signal         D5M_LVAL 	: std_logic;								-- Line Valid
signal         D5M_SCLK 	: std_logic;								-- I2C Clock
signal         D5M_SDATA 	: std_logic;								-- I2C Data
signal         D5M_RESET_n  : std_logic;								-- Reset	Active Low
signal         D5M_RESET	: std_logic;
signal         D5M_D 		: std_logic_vector(11 downto 0);		-- Pixel Data

-------------------------------Signals to connect Screen's Pins Through GPiO_0-----------------
	-- lcd 3wire interface
signal ltm_sda				: std_logic;								-- LCD 3-Wire Serial Interface Data I/O
signal ltm_scen				: std_logic;								-- LCD 3-Wire Serial Interface enable / ADC Chip Enable
signal ltm_3wirebusy_n		: std_logic;
signal ltm_sclk				: std_logic;
signal adc_ltm_sclk			: std_logic;
-- lcd display signals
signal ltm_nclk				: std_logic;								-- LCD Clock Input Signal
signal ltm_grst				: std_logic; 								-- LCD Global Reset (Active Low)
signal ltm_DATA_EN			: std_logic;								-- LCD RGB Data Enable 
signal ltm_hs				: std_logic;								-- LCD Horizontal Sync
signal ltm_vs				: std_logic;								-- LCD Vertical Sync 
signal LTM_R				: std_logic_vector(7 downto 0);
signal LTM_G				: std_logic_vector(7 downto 0);
signal LTM_B				: std_logic_vector(7 downto 0);

-- Touch Screen Digitizer ADC	
signal adc_dclk				: std_logic;
signal adc_penirq_n			: std_logic;
signal adc_din				: std_logic;
signal adc_dout				: std_logic;
signal adc_busy				: std_logic;

--------------------------------Signals To Connect the Bridge to SRAM/DRAM-------------------------

signal SRAMB_Add			: std_logic_vector(27 downto 0);
signal SRAMB_ByteEnable		: std_logic;
signal SRAMB_ReadEnable		: std_logic;
signal SRAMB_WriteEnable	: std_logic;
signal SRAMB_WriteData		: std_logic_vector(7 downto 0);
signal SRAMB_ACK			: std_logic;
signal SRAMB_ReadData		: std_logic_vector(7 downto 0);

signal DRAMB_Add			: std_logic_vector(27 downto 0);
signal DRAMB_ByteEnable		: std_logic;
signal DRAMB_ReadEnable		: std_logic;
signal DRAMB_WriteEnable	: std_logic	;
signal DRAMB_WriteData		: std_logic_vector(7 downto 0);
signal DRAMB_ACK			: std_logic;
signal DRAMB_ReadData		: std_logic_vector(7 downto 0);

------------------------------Signals to Concatenate DRAM between Physical Chip and Qsys One-------------
signal DRAM_ADDR	: STD_LOGIC_VECTOR(12 DOWNTO 0);
signal DRAM_BA		: STD_LOGIC_VECTOR(1 DOWNTO 0);
signal DRAM_CAS_N	: STD_LOGIC;
signal DRAM_RAS_N	: STD_LOGIC;
signal DRAM_CLK		: STD_LOGIC;
signal DRAM_CKE		: STD_LOGIC;
signal DRAM_CS_N	: STD_LOGIC;
signal DRAM_DQM		: STD_LOGIC_VECTOR(3 DOWNTO 0);
signal DRAM_WE_N	: STD_LOGIC;
---------------------------------------Video Characteristics----------------------------------------------
constant H_Resolution			:natural:=320;
constant V_Resolution			:natural:=240;
signal MP_Size					:integer:=2;
--constant MP_Size				:natural:=2;

---------------------------------------OCM for Pixels (HD)-------------------------------------------------
constant Pixel_OCM_Size 	:natural:=H_Resolution*V_Resolution;

type OCM_HD is array (0 to Pixel_OCM_Size-1) of std_logic_vector(7 downto 0);  -- Hold all the frame
signal Pixel_OCM: OCM_HD;
signal Pixel_index_R		:integer:=0;
signal Pixel_index_W		:integer:=0;
signal Pixel_DataIn			:std_logic_vector(7 downto 0);
signal Pixel_DataOut		:std_logic_vector(7 downto 0);
signal Pixel_Read_En		:std_logic;
signal Pixel_Write_En		:std_logic;
signal Pixel_index_Max		:integer:=Pixel_OCM_Size-1;

---------------------------------------OCM for MacroPixels_Average-------------------------------------------------

constant MP_Avg_OCM_Size 	:natural:=(H_Resolution/MP_Size)*(V_Resolution/MP_Size);
type OCM_AVG is array (0 to MP_Avg_OCM_Size-1) of std_logic_vector(7 downto 0);  -- Hold the Averaged Value 
signal MP_Avg_OCM: OCM_AVG;
signal MP_Avg_index_R		:integer:=0;
signal MP_Avg_index_W		:integer:=0;
signal MP_Avg_DataIn		:std_logic_vector(7 downto 0);
signal MP_Avg_DataOut		:std_logic_vector(7 downto 0);
signal MP_Avg_Read_En		:std_logic;
signal MP_Avg_Write_En		:std_logic;
signal MP_Avg_index_Max		:integer:=MP_Avg_OCM_Size-1;

signal MP_Avg_DataOut_Buffer:std_logic_vector(7 downto 0);

signal Read_Write_FSM_State	:integer:=0;

signal HD_flag				:std_logic;

Signal SramAddCount			:integer:=0; 				-- Address Counter for SRAM
Signal SramAddMax			:integer:=(2**19)-1;

Signal DramAddCount			:integer:=0;
Signal DramAddMax			:integer:=(2**19)-1;

signal A	:integer:=0;
signal N	:integer:=0;
signal X	:integer:=0;
--------------------------------LCD_SPI_Controller Used For Init -----------------------------------------------------------------------
component LCD_SPI_Controller
port (
	iCLK			: in std_logic;
	iRST_n			: in std_logic;
	o3WIRE_SCLK		: out std_logic;
	io3WIRE_SDAT	: inout std_logic;
	o3WIRE_SCEN		: out std_logic;
	o3WIRE_BUSY_n	: out std_logic
);
end component;
	
--------------------------------Qsys System Component-------------------------------------------------------------------------
component Qsys_System is
	  port (
			clk_clk                : in    std_logic                     := 'X';             -- 50Mhz Input
			reset_reset_n          : in    std_logic                     := 'X';             -- reset_n
			
			camera_PIXEL_CLK       : in    std_logic                     := 'X';             -- PIXEL_CLK
			camera_LINE_VALID      : in    std_logic                     := 'X';             -- LINE_VALID
			camera_FRAME_VALID     : in    std_logic                     := 'X';             -- FRAME_VALID
			camera_pixel_clk_reset : in    std_logic                     := 'X';             -- pixel_clk_reset
			camera_PIXEL_DATA      : in    std_logic_vector(11 downto 0) := (others => 'X'); -- PIXEL_DATA
			camera_config_SDAT     : inout std_logic                     := 'X';             -- SDAT
         camera_config_SCLK   	   : out   std_logic;                                        -- SCLK
			
			sram_DQ                : inout std_logic_vector(31 downto 0) := (others => 'X'); -- DQ
			sram_DPA               : inout std_logic_vector(3 downto 0)  := (others => 'X'); -- DPA
			sram_ADDR              : out   std_logic_vector(18 downto 0);                    -- ADDR
			sram_ADSC_N            : out   std_logic;                                        -- ADSC_N
			sram_ADSP_N            : out   std_logic;                                        -- ADSP_N
			sram_ADV_N             : out   std_logic;                                        -- ADV_N
			sram_BE_N              : out   std_logic_vector(3 downto 0);                     -- BE_N
			sram_CE1_N             : out   std_logic;                                        -- CE1_N
			sram_CE2               : out   std_logic;                                        -- CE2
			sram_CE3_N             : out   std_logic;                                        -- CE3_N
			sram_GW_N              : out   std_logic;                                        -- GW_N
			sram_OE_N              : out   std_logic;                                        -- OE_N
			sram_WE_N              : out   std_logic;                                        -- WE_N
			sram_CLK               : out   std_logic;                                        -- CLK
			
			bridge_sram_r_address  : in    std_logic_vector(27 downto 0) := (others => 'X'); -- address
         bridge_sram_r_byte_enable : in    std_logic                     := 'X';             -- byte_enable
         bridge_sram_r_read        : in    std_logic                     := 'X';             -- read
         bridge_sram_r_write       : in    std_logic                     := 'X';             -- write
         bridge_sram_r_write_data  : in    std_logic_vector(7 downto 0)  := (others => 'X'); -- write_data
         bridge_sram_r_acknowledge : out   std_logic;                                        -- acknowledge
         bridge_sram_r_read_data   : out   std_logic_vector(7 downto 0);                     -- read_data
			
         bridge_dram_w_address     : in    std_logic_vector(27 downto 0) := (others => 'X'); -- address
         bridge_dram_w_byte_enable : in    std_logic                     := 'X';             -- byte_enable
         bridge_dram_w_read        : in    std_logic                     := 'X';             -- read
         bridge_dram_w_write       : in    std_logic                     := 'X';             -- write
         bridge_dram_w_write_data  : in    std_logic_vector(7 downto 0)  := (others => 'X'); -- write_data
         bridge_dram_w_acknowledge : out   std_logic;                                        -- acknowledge
         bridge_dram_w_read_data   : out   std_logic_vector(7 downto 0);                     -- read_data
			
			sdram_addr             : out   std_logic_vector(12 downto 0);                    -- addr
			sdram_ba               : out   std_logic_vector(1 downto 0);                     -- ba
         sdram_cas_n               : out   std_logic;                                        -- cas_n
         sdram_cke                 : out   std_logic;                                        -- cke
         sdram_cs_n                : out   std_logic;                						      -- cs_n
         sdram_dq                  : inout std_logic_vector(31 downto 0) := (others => 'X'); -- dq
         sdram_dqm                 : out   std_logic_vector(3 downto 0);                     -- dqm
         sdram_ras_n               : out   std_logic;                                        -- ras_n
         sdram_we_n                : out   std_logic;                                        -- we_n
			
			lcd_CLK                : out   std_logic;                                        -- CLK
			lcd_HS                 : out   std_logic;                                        -- HS
			lcd_VS                 : out   std_logic;                                        -- VS
			lcd_BLANK              : out   std_logic;                                        -- BLANK
			lcd_SYNC               : out   std_logic;                                        -- SYNC
			lcd_DATA_EN            : out   std_logic;                                        -- DATA_EN
			lcd_R                  : out   std_logic_vector(7 downto 0);                     -- R
			lcd_G                  : out   std_logic_vector(7 downto 0);                     -- G
			lcd_B                  : out   std_logic_vector(7 downto 0);                     -- B
			
			lcd_clk_clk            : out   std_logic;                                        -- 25MHz
			camera_clk_clk         : out   std_logic                                         -- 96MHz
	  );
end component Qsys_System;

begin
--LEDR(0)<='1';

MP_Size<=to_integer(unsigned(SW(17 downto 10)));
Clock_System 		<= clk_96;							-- Assign Clock System to use 96MHz from PLL
DRAM_CLK			<= clk_96;
RESET				<= KEY(0);							-- Assign Reset to be Key(0)


--------------------------------------------- Beginning of Algorithm----------------------------------------------

-----------------------------------------------Initializing-----------------------------------------------
READ_SRAM_Write_DRAM: process(Clock_System, RESET) is

begin
	if RESET='0'  then
	
		Pixel_DataIn		<=(others => '0');
		Pixel_index_W 		<=0;
		Pixel_Write_En		<='0';
		Pixel_index_R 		<=0;
		Pixel_Read_En		<='0';
		
		SramADDCount	  	<=0;
		SRAMB_Add			<=(others => '0');
		SRAMB_ReadEnable	<='0';
		SRAMB_WriteEnable	<='0';
		
		DramADDCount	  	<=0;
		DRAMB_ReadEnable	<='0';
		DRAMB_WriteEnable	<='0';
		DRAMB_Add			<=(others => '0');
		
		Read_Write_FSM_State  	<=0;
		
	elsif rising_edge(Clock_System) then
		Case Read_Write_FSM_State is
	
		when 0 => 
			
			Pixel_DataIn		<=(others => '0');
			Pixel_index_W 		<=0;
			Pixel_Write_En		<='0';
			Pixel_index_R 		<=0;
			Pixel_Read_En		<='0';
			
			SramADDCount	  	<=0;
			SRAMB_Add			<=(others => '0');
			SRAMB_ReadEnable	<='0';
			SRAMB_WriteEnable	<='0';
			
			DramADDCount	  	<=0;
			DRAMB_ReadEnable	<='0';
			DRAMB_WriteEnable	<='0';
			DRAMB_Add			<=(others => '0');
			
			MP_Avg_Read_En		<='0';
			MP_Avg_Write_En		<='0';
			MP_Avg_index_R		<=0;
			MP_Avg_index_W		<=0;	
			MP_Avg_DataIn		<=(others => '0');
		
			Read_Write_FSM_State  	<=12;
			
		when 12 =>
			if (HD_flag='1') then
				Read_Write_FSM_State <=13;		--Go to HD 
				HEX0<="11000000";
				HEX1<="10001001";
			else
				Read_Write_FSM_State <=1;		--Go to SD(Compute Average and store it)
				HEX0<="11000000";
				HEX1<="10010010";
			end if;	
	
---------------------------------------------Read From SRAM, Compute Avg, store in MP_Avg_OCM-----------------------------------				
		when 1 =>
			SRAMB_ByteEnable 			<='1';
			SRAMB_ReadEnable			<='1';
			SRAMB_Add					<="100000000" & std_logic_vector(to_unsigned(SramAddCount,19));
			MP_Avg_Write_En				<='1';
			
			Read_Write_FSM_State 		<=2;
		when 2 =>
			
			if (SRAMB_ACK='1') then 
											-------Cumulative Recursive Average ==>Y(n+1)=(Y(n)*n+X(n))/(n+1) --------
				MP_Avg_DataIn<=std_logic_vector(to_unsigned(((to_integer(unsigned(MP_Avg_DataIn))*N)+to_integer(unsigned(SRAMB_ReadData)))/(N+1),8));
				SRAMB_ReadEnable 		<='0';
				MP_Avg_Write_En		<='0';
				Read_Write_FSM_State	<=3;
			else
				Read_Write_FSM_State	<=2;
			end if;

		when 3 =>
			if(SramAddCount<=Pixel_index_max)							then
				if((SramAddCount+1) mod (H_Resolution*MP_Size)/=0) 			then
					if((SramAddCount+1) mod H_Resolution /=0)					then
						if((SramAddCount+1) mod MP_Size /=0)						then
							N								<=N+1;
							SramAddCount 				<=SramAddCount+1;
							MP_Avg_index_W				<=MP_Avg_index_W;
							Read_Write_FSM_State  	<=1;
						else
							SramAddCount 				<=SramAddCount+1;
							MP_Avg_index_W				<=MP_Avg_index_W+1;
							N								<=0+A;									
							Read_Write_FSM_State  	<=1;
						end if;
					else
						SramAddCount 				<=SramAddCount+1;
						MP_Avg_index_W				<=X;								
						A								<=A+MP_Size;	
						Read_Write_FSM_State  	<=1;
					end if;
				else
					SramAddCount 				<=SramAddCount+1;
					X							<=X+(H_Resolution/MP_Size);
					MP_Avg_index_W				<=X;
					A							<=0;
					N							<=0;
					Read_Write_FSM_State 	 	<=1;
				end if;
			else
				SramAddCount 				<=0;
				MP_Avg_index_W				<=0;
				A								<=0;
				N								<=0;
				X								<=0;
				Read_Write_FSM_State		<=4;
			end if;

						--------Transfert MacroPixel Average OCM(H_Resolution/MP_Size*V_Resolution/MP_Size) to Pixel OCM (H_Resolution*V_Resolution)-------			

		when 4 =>
			MP_Avg_Read_En				<='1';
			Pixel_Write_En				<='0';
			Read_Write_FSM_State  		<=5;
		when 5 =>
			MP_Avg_DataOut_Buffer		<=MP_Avg_DataOut;  
			MP_Avg_Read_En				<='0';
			Pixel_Write_En				<='1';
			Read_Write_FSM_State  		<=11;
		when 11 =>
			Pixel_DataIn				<=MP_Avg_DataOut_Buffer;
			MP_Avg_Read_En				<='0';
			Pixel_Write_En				<='0';
			Read_Write_FSM_State  		<=6;
		when 6 =>
			if(Pixel_index_W<=Pixel_index_max)							then
				if((Pixel_index_W+1) mod (H_Resolution*MP_Size)/=0)	then
					if((Pixel_index_W+1) mod H_Resolution /=0)				then
						if((Pixel_index_W+1) mod MP_Size /=0)						then
							Pixel_index_W				<=Pixel_index_W+1;
							MP_Avg_index_R				<=MP_Avg_index_R;
							Read_Write_FSM_State 	 	<=4;
						else
							Pixel_index_W				<=Pixel_index_W+1;
							MP_Avg_index_R				<=MP_Avg_index_R+1;
							Read_Write_FSM_State  		<=4;
						end if;
					else
						Pixel_index_W				<=Pixel_index_W+1;	
						MP_Avg_index_R				<=X;
						Read_Write_FSM_State  		<=4;
					end if;
				else
					Pixel_index_W					<=Pixel_index_W+1;	
					X								<=X+(H_Resolution/MP_Size);
					MP_Avg_index_R					<=X;
					
					Read_Write_FSM_State  		<=4;
				end if;
			else
				MP_Avg_Read_En					<='0';
				Pixel_Write_En					<='0';
				X								<=0;
				Read_Write_FSM_State			<=7;			--Go to Write on SDRAM
			end if;

---------------------------------------------Read From SRAM, store in Pixel_OCM for HD Output----------------------------------------------
		when 13 =>
			SRAMB_ByteEnable 			<='1';
			SRAMB_ReadEnable			<='1';
			SRAMB_Add					<="100000000" & std_logic_vector(to_unsigned(SramAddCount,19));
			Pixel_Write_En				<='1';
			Read_Write_FSM_State 		<=14;
		when 14 =>
			if (SRAMB_ACK='1') then 
				Pixel_DataIn 			<=SRAMB_ReadData;
				SRAMB_ReadEnable 		<='0';
				Pixel_Write_En			<='0';
				Read_Write_FSM_State	<=15;
			else
				Read_Write_FSM_State	<=14;
			end if;
		
		when 15 =>
			
			if(Pixel_index_W<=Pixel_index_Max) then
				SramAddCount 				<=SramAddCount+1;
				Pixel_index_W				<=Pixel_index_W+1;
				Read_Write_FSM_State  	<=13;
			elsif(Pixel_index_W>Pixel_index_Max) then 
				SramAddCount 				<=0;
				Pixel_index_W				<=0;
				Read_Write_FSM_State  	<=7;				-- Go to Write on SDRAM
			end if;							
-------------------------------------------------Write on SDRAM---------------------------------
		when 7 =>
			DRAMB_ByteEnable 		<='1';
			DRAMB_WriteEnable		<='1';
			DRAMB_Add				<="000000000" & std_logic_vector(to_unsigned(DramAddCount,19));
			Pixel_Read_En			<='1';
			Read_Write_FSM_State	<=8;
		when 8 =>
			DRAMB_WriteData		<=Pixel_DataOut;
			Read_Write_FSM_State	<=9;

		when 9 =>
			if (DRAMB_ACK='1') then 
				Pixel_Read_En			<='0';
				DRAMB_WriteEnable		<='0';
				Read_Write_FSM_State	<=10;
			else
				Read_Write_FSM_State	<=9;
			end if;
		when 10 =>
			if(Pixel_index_R<=Pixel_index_Max ) then
				DramAddCount 			<=DramAddCount+1;
				Pixel_index_R			<=Pixel_index_R+1;
				Read_Write_FSM_State	<=7;
			elsif(Pixel_index_R>Pixel_index_Max ) then 
				DramAddCount 			<=0;
				Pixel_index_R			<=0;
				Read_Write_FSM_State	<=0;
			end if;
			
		when others =>
				Read_Write_FSM_State	<=0;
		end case;
	end if;
end process;
----------------------------------------------End of Algorithm-----------------------------------------------------

------------------------------------------------------Multiplexing between HD and SD-------------------------
Mux_HD_SD: process(clock_System) is
begin
	if (SW(0)='1') then
		HD_Flag<='1';
	else 
		HD_Flag<='0';
	end if;
end process;
	

---------------------------------------------Pixel_OCM_Process-----------------------------------------------------
Pixel_OCM_Process : PROCESS(Clock_System) IS
	BEGIN
		IF rising_edge(Clock_System) THEN
			IF Pixel_Write_En = '1' THEN       
				Pixel_OCM(Pixel_index_W) <= Pixel_DataIn;
			ELSIF Pixel_Read_En = '1' THEN 
				Pixel_DataOut <= Pixel_OCM(Pixel_index_R);
			END IF;
		END IF; 
	END PROCESS;

---------------------------------------------MacroPixel_Average_OCM_Process-----------------------------------------------------
MacroPixel_Average_OCM_Process : PROCESS(Clock_System) IS
	BEGIN
		IF rising_edge(Clock_System) THEN
			IF MP_Avg_Write_En = '1' THEN       
				MP_Avg_OCM(MP_Avg_index_W) <= MP_Avg_DataIn;
			ELSIF MP_Avg_Read_En = '1' THEN 
				MP_Avg_DataOut <= MP_Avg_OCM(MP_Avg_index_R);
			END IF;
		END IF; 
	END PROCESS;	

--------------------------------Qsys System Instantiation ---------------------------------------------------------
Qsys : component Qsys_System
         port map (
            clk_clk                => CLOCK_50,		    			  	            --        clk.clk
            reset_reset_n          => RESET,         								--      reset.reset_n
				
            camera_PIXEL_CLK       => D5M_PIXCLK,      								--     camera.PIXEL_CLK
            camera_LINE_VALID      => D5M_LVAL,      								--           .LINE_VALID
            camera_FRAME_VALID     => D5M_FVAL,     								--           .FRAME_VALID
            camera_pixel_clk_reset => D5M_RESET, 									--           .pixel_clk_reset
            camera_PIXEL_DATA      => D5M_D,      									--           .PIXEL_DATA
			camera_config_SDAT     => D5M_SDATA,     								-- camera_config.SDAT
            camera_config_SCLK     => D5M_SCLK,    									--              .SCLK
				
			sram_DQ                => SRAM_DQ,     	          --     	    sram.DQ
            sram_DPA               => SRAM_DPA,               --           .DPA
            sram_ADDR              => SRAM_ADDR,              --           .ADDR
            sram_ADSC_N            => SRAM_ADSC_N,            --           .ADSC_N
            sram_ADSP_N            => SRAM_ADSP_N,            --           .ADSP_N
            sram_ADV_N             => SRAM_ADV_N,             --           .ADV_N
            sram_BE_N              => SRAM_BE_N,              --           .BE_N
            sram_CE1_N             => SRAM_CE1_N,             --           .CE1_N
            sram_CE2               => SRAM_CE2,               --           .CE2
            sram_CE3_N             => SRAM_CE3_N,             --           .CE3_N
            sram_GW_N              => SRAM_GW_N,              --           .GW_N
            sram_OE_N              => SRAM_OE_N,              --           .OE_N
            sram_WE_N              => SRAM_WE_N,              --           .WE_N
            sram_CLK               => SRAM_CLK,               --           .CLK
				
			--TO DO: Figure out what this bridge sram stuff is...
			bridge_sram_r_address     => SRAMB_Add,     			--				 bridge_sram_r.address
            bridge_sram_r_byte_enable => SRAMB_ByteEnable,		 	--              .byte_enable
            bridge_sram_r_read        => SRAMB_ReadEnable,          --              .read
            bridge_sram_r_write       => SRAMB_WriteEnable,         --              .write
            bridge_sram_r_write_data  => SRAMB_WriteData,  			--              .write_data
            bridge_sram_r_acknowledge => SRAMB_ACK,  				--              .acknowledge
            bridge_sram_r_read_data   => SRAMB_ReadData,   			--              .read_data
				
            bridge_dram_w_address     => DRAMB_Add,     		    -- 			    bridge_dram_w.address
            bridge_dram_w_byte_enable => DRAMB_ByteEnable, 		    --			   .byte_enable
            bridge_dram_w_read        => DRAMB_ReadEnable,          --             .read
            bridge_dram_w_write       => DRAMB_WriteEnable,         --             .write
            bridge_dram_w_write_data  => DRAMB_WriteData,  		    --			   .write_data
            bridge_dram_w_acknowledge => DRAMB_ACK, 			    -- 			   .acknowledge
            bridge_dram_w_read_data   => DRAMB_ReadData,  		    --             .read_data
				
			sdram_addr                => DRAM_ADDR,          	    --	            sdram.addr
            sdram_ba                  => DRAM_BA,                   --             .ba
            sdram_cas_n               => DRAM_CAS_N,                --             .cas_n
            sdram_cke                 => DRAM_CKE,                  --             .cke
            sdram_cs_n                => DRAM_CS_N,                 --             .cs_n
            sdram_dq                  => DRAM_DQ,                   --             .dq
            sdram_dqm                 => DRAM_DQM,                  --             .dqm
            sdram_ras_n               => DRAM_RAS_N,                --             .ras_n
            sdram_we_n                => DRAM_WE_N,                 --             .we_n
				
            lcd_HS                 => LTM_HS,                 --           .HS
            lcd_VS                 => LTM_VS,                 --           .VS
			lcd_DATA_EN            => LTM_DATA_EN,   	      --           .DATA_EN
            lcd_R                  => LTM_R,                  --           .R
            lcd_G                  => LTM_G,                  --           .G
            lcd_B                  => LTM_B,                  --           .B			
            lcd_clk_clk            => Clk_25,           	  --    	   lcd_clk.clk
            camera_clk_clk         => Clk_96          		  -- 		   camera_clk.clk
        );
-------------------------------End Qsys Instantiation--------------------------------------

--------------------------------LCD_SPI_Controller_Instantiation-----------------------
	LCD_Init: LCD_SPI_Controller
	port map (
		-- Host Side
		iCLK		=> CLOCK_50,
		iRST_n	=> RESET,
		-- 3wire Side
		o3WIRE_SCLK		=> ltm_sclk,
		io3WIRE_SDAT	=> ltm_sda,
		o3WIRE_SCEN		=> ltm_scen,
		o3WIRE_BUSY_n	=> ltm_3wirebusy_n
	);
-------------------------------End LCD_SPI_Controller_Instantiation-------------------

----------------------------Camera To GPIO_1---------------------------------
	D5M_PIXCLK	<= GPIO_CLKIN_N1;
	GPIO_CLKOUT_N1	<= D5M_XCLKIN;
    D5M_D(11) <=  GPIO_1(0);
	D5M_D(10) <=  GPIO_1(1);
	D5M_D(9) <=  GPIO_1(2);
	D5M_D(8) <=  GPIO_1(3);
	D5M_D(7) <=  GPIO_1(4);
	D5M_D(6) <=  GPIO_1(5);
	D5M_D(5) <=  GPIO_1(6);
	D5M_D(4) <=  GPIO_1(7);
	D5M_D(3) <=  GPIO_1(8);
	D5M_D(2) <=  GPIO_1(9);
	D5M_D(1) <=  GPIO_1(10);
	D5M_D(0) <=  GPIO_1(11);
	D5M_FVAL <=  GPIO_1(18);
	D5M_LVAL <=  GPIO_1(17);
	GPIO_1(14) <= D5M_RESET_n;			--Tocheck
	GPIO_1(20) <= D5M_SCLK;
	GPIO_1(19) <= D5M_SDATA;
	GPIO_1(15) <= D5M_TRIGGER;
	GPIO_1(16) <= D5M_STROBE;
----------------------------End Camera To GPIO_1-------------------------------

---------------------- Init Camera ---------------------------------	
	D5M_STROBE  <= '0';
	D5M_TRIGGER <= '1';
	D5M_RESET_n <= RESET; 				----------------- Tocheck	Reset Using Push Button KEY(0)
	D5M_XCLKIN  <= Clk_96; 				----------------- Tocheck 96MHz to drive the Camera
	D5M_RESET   <= not D5M_RESET_n;
	
----------------------- GPIO_0 To LCD -------------------------------
	adc_penirq_n		<= GPIO_CLKIN_N0;
	adc_dout			<= GPIO_1(0);
	adc_busy			<= GPIO_CLKIN_P0;
	GPIO_0(1)			<= adc_din;
	GPIO_0(2)			<= adc_ltm_sclk;
	GPIO_0(3)			<= LTM_B(3);
	GPIO_0(4)			<= LTM_B(2);
	GPIO_0(5)			<= LTM_B(1);
	GPIO_0(6)			<= LTM_B(0);
	GPIO_0(7)			<= LTM_nclk;						-- Tocheck
	GPIO_0(8)			<= LTM_DATA_EN;
	GPIO_0(9)			<= LTM_hs;
	GPIO_0(10)			<= LTM_vs;
	GPIO_0(11)			<= LTM_B(4);
	GPIO_0(12)			<= LTM_B(5);
	GPIO_0(13)			<= LTM_B(6);
	GPIO_CLKOUT_N0		<= LTM_B(7);
	GPIO_0(14)			<= LTM_G(0);
	GPIO_CLKOUT_P0		<= LTM_G(1);
	GPIO_0(15)			<= LTM_G(2);
	GPIO_0(16)			<= LTM_G(3);
	GPIO_0(17)			<= LTM_G(4);
	GPIO_0(18)			<= LTM_G(5);
	GPIO_0(19)			<= LTM_G(6);
	GPIO_0(20)			<= LTM_G(7);
	GPIO_0(21)			<= LTM_R(0);
	GPIO_0(22)			<= LTM_R(1);
	GPIO_0(23)			<= LTM_R(2);
	GPIO_0(24)			<= LTM_R(3);
	GPIO_0(25)			<= LTM_R(4);
	GPIO_0(26)			<= LTM_R(5);
	GPIO_0(27)			<= LTM_R(6);
	GPIO_0(28)			<= LTM_R(7);
	GPIO_0(29)			<= LTM_Grst;
	GPIO_0(30)			<= LTM_scen;
	GPIO_0(31)			<= LTM_sda;
	
	adc_ltm_sclk	<= (adc_dclk and ltm_3wirebusy_n) or (not(ltm_3wirebusy_n) and ltm_sclk);
	LTM_nclk 	    <=clk_25;							-- 25 MHz To Drive the LCD
	LTM_Grst        <=RESET;							-- Use Push Button '0' to reset the LCD
----------------------------End LCD To GPIO_0-------------------------------	

------------------------------ SDRAM_Controller to SDRAM Chip------------------------------
DRAM0_ADDR	<= DRAM_ADDR;
DRAM0_BA_1	<= DRAM_BA(1);
DRAM0_BA_0	<= DRAM_BA(0);
DRAM0_CAS_N	<= DRAM_CAS_N;
DRAM0_CKE	<= DRAM_CKE;
DRAM0_CLK	<= DRAM_CLK;
DRAM0_CS_N	<= DRAM_CS_N;
DRAM0_RAS_N	<= DRAM_RAS_N;
DRAM0_WE_N	<= DRAM_WE_N;
DRAM0_UDQM	<= DRAM_DQM(1);
DRAM0_LDQM	<= DRAM_DQM(0);

DRAM1_ADDR	<= DRAM_ADDR;
DRAM1_BA_1	<= DRAM_BA(1);
DRAM1_BA_0	<= DRAM_BA(0);
DRAM1_CAS_N	<= DRAM_CAS_N;
DRAM1_CKE	<= DRAM_CKE;
DRAM1_CLK	<= DRAM_CLK;
DRAM1_CS_N	<= DRAM_CS_N;
DRAM1_RAS_N	<= DRAM_RAS_N;
DRAM1_WE_N	<= DRAM_WE_N;
DRAM1_UDQM	<= DRAM_DQM(3);
DRAM1_LDQM	<= DRAM_DQM(2);
-------------------------------End Bridge To SDRAM---------------------------
LEDG(0)<='1';	
end RTL;

