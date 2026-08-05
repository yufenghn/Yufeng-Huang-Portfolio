
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity test_alu is
--  Port ( );
end test_alu;

architecture Behavioral of test_alu is

component ALU
Port ( a : in STD_LOGIC_VECTOR (5 downto 0);
           b : in STD_LOGIC_VECTOR (5 downto 0);
           aluop : in STD_LOGIC_VECTOR (2 downto 0);
           zero : out STD_LOGIC; -- zero indicator.
           res : out STD_LOGIC_VECTOR (5 downto 0);
           cout: out STD_LOGIC; -- carry out
           ofl : out STD_LOGIC); --overflow
end component;

constant global_wait : time := 100ns;

signal a,b, res: std_logic_vector(5 downto 0);
signal aluop: std_logic_vector(2 downto 0);
signal zero, cout, ofl: std_logic;

begin

uutaluall: ALU port map(
a => a,
b => b,
aluop => aluop,
zero => zero,
res => res,
cout => cout,
ofl => ofl
);

-- Please Note: These are just placeholder values given as reference. 
-- Actual values may be different during evaluation of the assignment.
 process begin
    wait for global_wait;
    a <= "110101"; b <= "100101"; aluop <= "000"; wait for 20 ns;
    a <= "010101"; b <= "011111"; aluop <= "000"; wait for 20 ns;
    a <= "100001"; b <= "111110"; aluop <= "010"; wait for 20 ns;
    a <= "110101"; b <= "111110"; aluop <= "010"; wait for 20 ns;
    a <= "010010"; b <= "101000"; aluop <= "111"; wait for 20 ns;
    a <= "111111"; b <= "001100"; aluop <= "101"; wait for 20 ns;
    a <= "010010"; b <= "101000"; aluop <= "110"; wait for 20 ns;
    a <= "111111"; b <= "001100"; aluop <= "100"; wait for 20 ns;
    wait;

 end process;

end Behavioral;