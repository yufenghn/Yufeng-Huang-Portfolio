library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity full_adder is
    Port (
        a    : in  STD_LOGIC;
        b    : in  STD_LOGIC;
        Cin  : in  STD_LOGIC;
        sum  : out STD_LOGIC;
        cout : out STD_LOGIC
    );
end full_adder;

architecture gate_level of full_adder is
begin
    sum  <= a XOR b XOR Cin;
    cout <= (a AND b) OR (Cin AND a) OR (Cin AND b);
end gate_level;


