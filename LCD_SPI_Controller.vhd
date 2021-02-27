-- Converted from lcd_spi_cotroller.v
-- by Verilog2VHDL ver1.00(2004/05/06)  Copyright(c) S.Morioka (http://www02.so-net.ne.jp/~morioka/v2v.htm)


-- --------------------------------------------------------------------
-- Copyright (c) 2005 by Terasic Technologies Inc. 
-- --------------------------------------------------------------------
--
-- Permission:
--
--   Terasic grants permission to use and modify this code for use
--   in synthesis for all Terasic Development Boards and Altera Development 
--   Kits made by Terasic.  Other use of this code, including the selling 
--   ,duplication, or modification of any portion is strictly prohibited.
--
-- Disclaimer:
--
--   This VHDL/Verilog or C/C++ source code is intended as a design reference
--   which illustrates how these types of functions can be implemented.
--   It is the user's responsibility to verify their design for
--   consistency and functionality through the use of formal
--   verification methods.  Terasic provides no warranty regarding the use 
--   or functionality of this code.
--
-- --------------------------------------------------------------------
--           
--                     Terasic Technologies Inc
--                     356 Fu-Shin E. Rd Sec. 1. JhuBei City,
--                     HsinChu County, Taiwan
--                     302
--
--                     web: http://www.terasic.com/
--                     email: support@terasic.com
--
-- --------------------------------------------------------------------
--
-- Major Functions:	This function will transmit the lcd register setting 
--            
-- --------------------------------------------------------------------
--
-- Revision History :
-- --------------------------------------------------------------------
--   Ver  :| Author            		:| Mod. Date :| Changes Made:
--   V1.0 :| Johnny Fan				:| 07/06/30  :| Initial Revision
-- --------------------------------------------------------------------
--	Host Side
--	3wire interface side
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity LCD_SPI_Controller is
	port (
	--===========================================================================
	-- PORT declarations
	--===========================================================================
	--	Host Side
		o3WIRE_BUSY_n	: out std_logic;
		iCLK	: in  std_logic;
		iRST_n	: in  std_logic;
	--	3wire interface side
		o3WIRE_SCLK	: out std_logic;
		io3WIRE_SDAT	: inout std_logic;
		o3WIRE_SCEN	: out std_logic 
	);
end LCD_SPI_Controller;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

architecture RTL of LCD_SPI_Controller is

-- WARNING(0): Please modify the following component declaration manually.
	component Three_Wire_Controller
	port (
		iCLK	: in std_logic;
		iRST	: in std_logic;
		iDATA	: in std_logic_vector(15 downto 0);
		iSTR	: in std_logic;
		oACK	: out std_logic;
		oRDY	: out std_logic;
		oCLK	: out std_logic;
		oSCEN	: out std_logic;
		SDA	: inout std_logic;
		oSCLK	: out std_logic
	);
	end component;

--============================================================================
-- PARAMETER declarations
--============================================================================						

-- WARNING(5) in line 54: Please write a signal width part in the following sentence, manually.
	constant LUT_SIZE	: integer := 20;	-- Total setting register numbers 
	--	Internal Registers/Wires
	--=============================================================================
	-- REG/WIRE declarations
	--=============================================================================
	signal m3wire_str	: std_logic;
	signal m3wire_rdy	: std_logic;
	signal m3wire_ack	: std_logic;
	signal m3wire_clk	: std_logic;
	signal m3wire_data	: std_logic_vector(15 downto 0);
	signal lut_data	: std_logic_vector(15 downto 0);
	signal lut_index	: unsigned(5 downto 0);
	signal msetup_st	: std_logic_vector(3 downto 0);
	signal v_reverse	: std_logic;	-- display Vertical reverse function
	signal h_reverse	: std_logic;	-- display Horizontal reverse function
	signal g0	: std_logic_vector(9 downto 0);
	signal g1	: std_logic_vector(9 downto 0);
	signal g2	: std_logic_vector(9 downto 0);
	signal g3	: std_logic_vector(9 downto 0);
	signal g4	: std_logic_vector(9 downto 0);
	signal g5	: std_logic_vector(9 downto 0);
	signal g6	: std_logic_vector(9 downto 0);
	signal g7	: std_logic_vector(9 downto 0);
	signal g8	: std_logic_vector(9 downto 0);
	signal g9	: std_logic_vector(9 downto 0);
	signal g10	: std_logic_vector(9 downto 0);
	signal g11	: std_logic_vector(9 downto 0);

	--=============================================================================
	-- Structural coding
	--=============================================================================


begin

	h_reverse	<= '1';
	v_reverse	<= '0';

	u0: three_wire_controller
	port map (
		--	Host Side
		iCLK	=> iCLK,
		iRST	=> iRST_n,
		iDATA	=> m3wire_data,
		iSTR	=> m3wire_str,
		oACK	=> m3wire_ack,
		oRDY	=> m3wire_rdy,
		oCLK	=> m3wire_clk,
		--	Serial Side
		oSCEN	=> o3WIRE_SCEN,
		SDA	=> io3WIRE_SDAT,
		oSCLK	=> o3WIRE_SCLK
	);
	--////////////////////	Config Control	////////////////////////////
	v2v_pr_0:process (m3wire_clk, iRST_n)
	begin
		if (not (iRST_n = '1')) then
			lut_index	<= "000000";
			msetup_st	<= X"0";
			m3wire_str	<= '0';
			o3WIRE_BUSY_n	<= '0';
		elsif (m3wire_clk'event and m3wire_clk = '1') then
			if (lut_index < to_unsigned(LUT_SIZE,6)) then
				o3WIRE_BUSY_n	<= '0';
				case (msetup_st) is
					when X"0" =>
						msetup_st	<= X"1";
					when X"1" =>
						msetup_st	<= X"2";

					when X"2" =>
						m3wire_data	<= lut_data;
						m3wire_str	<= '1';
						msetup_st	<= X"3";
					when X"3" =>
						if (m3wire_rdy = '1') then
							if (m3wire_ack = '1') then
								msetup_st	<= X"4";
							else
								msetup_st	<= X"0";
							end if;
							m3wire_str <= '0';
						end if;
					when X"4" =>
						lut_index <= lut_index + 1;
						msetup_st <= X"0";
					when others =>
					    msetup_st <= X"0";
				end case;
			else
				o3WIRE_BUSY_n	<= '1';
			end if;
		end if;
	end process;


	g0	<= std_logic_vector(to_unsigned(106,10));
	g1	<= std_logic_vector(to_unsigned(200,10));
	g2	<= std_logic_vector(to_unsigned(289,10));
	g3	<= std_logic_vector(to_unsigned(375,10));
	g4	<= std_logic_vector(to_unsigned(460,10));
	g5	<= std_logic_vector(to_unsigned(543,10));
	g6	<= std_logic_vector(to_unsigned(625,10));
	g7	<= std_logic_vector(to_unsigned(705,10));
	g8	<= std_logic_vector(to_unsigned(785,10));
	g9	<= std_logic_vector(to_unsigned(864,10));
	g10	<= std_logic_vector(to_unsigned(942,10));
	g11	<= std_logic_vector(to_unsigned(1020,10));

	--///////////////////	Config Data LUT	  //////////////////////////	
	v2v_pr_1:process (lut_index)
	begin
		case ( to_integer(lut_index)) is
		when 0 =>
			lut_data	<= ("01" & X"1" & "01" & g0(9 downto 8) & g1(9 downto 8) & g2(9 downto 8) & g3(9 downto 8));
		when 1 =>
			lut_data	<= ("01" & X"2" & "01" & g4(9 downto 8) & g5(9 downto 8) & g6(9 downto 8) & g7(9 downto 8));
		when 2 =>
			lut_data	<= ("01" & X"3" & "01" & g8(9 downto 8) & g9(9 downto 8) & g10(9 downto 8) & g11(9 downto 8));
		when 3 =>
			lut_data	<= ("01" & X"4" & "01" & g0(7 downto 0));
		when 4 =>
			lut_data	<= ("01" & X"5" & "01" & g1(7 downto 0));
		when 5 =>
			lut_data	<= ("01" & X"6" & "01" & g2(7 downto 0));
		when 6 =>
			lut_data	<= ("01" & X"7" & "01" & g3(7 downto 0));
		when 7 =>
			lut_data	<= ("01" & X"8" & "01" & g4(7 downto 0));
		when 8 =>
			lut_data	<= ("01" & X"9" & "01" & g5(7 downto 0));
		when 9 =>
			lut_data	<= ("01" & X"a" & "01" & g6(7 downto 0));
		when 10 =>
			lut_data	<= ("01" & X"b" & "01" & g7(7 downto 0));
		when 11 =>
			lut_data	<= ("01" & X"c" & "01" & g8(7 downto 0));
		when 12 =>
			lut_data	<= ("01" & X"d" & "01" & g9(7 downto 0));
		when 13 =>
			lut_data	<= ("01" & X"e" & "01" & g10(7 downto 0));
		when 14 =>
			lut_data	<= ("01" & X"f" & "01" & g11(7 downto 0));
		when 15 =>
			lut_data	<= ("10" & X"0" & "01" & X"f" & X"0");
		when 16 =>
			lut_data	<= ("10" & X"1" & "01" & X"f" & X"0");
		when 17 =>
			lut_data	<= ("00" & X"3" & "01" & X"df");
		when 18 =>
			lut_data	<= ("00" & X"2" & "01" & X"07");
		when 19 =>
			lut_data	<= ("00" & X"4" & "01000101" & not(v_reverse) & not(h_reverse ) );
		when others =>
			lut_data	<= X"0000";
		end case;
	end process;

end RTL;
