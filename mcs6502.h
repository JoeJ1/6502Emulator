/*  A simple emulation of the MCS6502 processor
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  Datasheet         : bit.ly/3jhddoA
 *  Hardware Manual   :   ""  /3dfX52v
 *  Programming Manual:   ""  /3qucaD0
 * 	Obelisk Reference :   ""  /3jk29H2
 *  6502.org          :   ""  /ex1jlzP
 *
 *    	                     Pin Diagram
 *    	                     ~~~~~~~~~~~
 *
 *            MCS6502 PINS                   KEY
 *           ______________           _______________________________________
 *          -| 1       40 |- RES      |  RDY  - Ready Signal                |
 *      RDY -| 2       39 |- t2(OUT)  |  t0-2 - Clocks                      |
 *   t1(OUT)-| 3       38 |- S.O.     |  IRQ  - Interrupt Request           |
 *      IRQ -| 4       37 |- t0(IN)   |  NMI  - Non-Maskable Interrupt      |
 *          -| 5       36 |-          |  SYNC - For single instruction      |
 *      NMI -| 6       35 |-          |         execution                   |
 *     SYNC -| 7       34 |- R/W      | AB0-15- Address Bus                 |
 *          -| 8       33 |- DB0      | DB0-7 - Bi-Directional Data bus.    |
 *      AB0 -| 9       32 |- DB1      |  R/W  - Read Write Bit For Data Bus |
 *      AB1 -| 10      31 |- DB2      |  S.O  - Set Overflow                |
 *      AB2 -| 11      30 |- DB3      |  RES  - Reset Signal                |
 *      AB3 -| 12      29 |- DB4      |       - Not Connected or not        |
 *      AB4 -| 13      28 |- DB5      |         relevant to emulation       |
 *      AB5 -| 14      27 |- DB6      ---------------------------------------
 *      AB6 -| 15      26 |- DB7
 *      AB7 -| 16      25 |- AB15     Note: Pins will not be emulated exactly
 *      AB8 -| 17      24 |- AB14           but it's useful to keep in mind
 *      AB9 -| 18      23 |- AB13           how the actual hardware operates.
 *     AB10 -| 19      22 |- AB12
 *     AB11 -| 20      21 |-
 *           --------------
 *
 *
 *    	               Programming Model
 *    	               ~~~~~~~~~~~~~~~~~
 *
 *     15       7       0
 *              [   A   ]         Accumulator         A
 *              [   Y   ]         Index Register      Y
 *              [   X   ]         Index Register      X
 *     [  PCH  ][  PCL  ]         Program Counter    "PC"
 *           [1][   S   ]         Stack Pointer      "S"
 *
 *     7                      0
 *     [N][V][U][B][D][I][Z][C]   Process Status Reg "P"
 *      |  |  |  |  |  |  |  |
 *      |  |  |  |  |  |  |  --> Carry (1=True)
 *      |  |  |  |  |  |  -----> Zero (1=Result 0)
 *      |  |  |  |  |  --------> IRQ Disable (1=Disable)
 *      |  |  |  |  -----------> Decimal Mode (1=True)
 *      |  |  |  --------------> BRK Command
 *      |  |  -----------------> Unused
 *      |  --------------------> Overflow (1=True)
 *      -----------------------> Negative (1=Neg)
 */

#include <cstdint> // Fixed width integer types
#include <map> // For opcode map

// Process Status Bits
#define PSC 0x01 // Carry (1 = True)
#define PSZ 0x02 // Zero (1 = Result 0)
#define PSI 0x04 // IRQ Disable (1 = Disable)
#define PSD 0x08 // Decimal Mode (1 = True)
#define PSB 0x10 // BRK Command
#define PSU 0x20 // Unused
#define PSV 0x40 // Overflow (1 = True)
#define PSN 0x80 // Negative (1 = Neg)

// Addressing Modes
#define IMM 0x01 // Immediate
#define ABS 0x02 // Absolute
#define ZPG 0x03 // Zero Page
#define ACC 0x04 // Accumulator
#define IMP 0x05 // Implied
#define IDX 0x06 // Indirect X
#define IDY 0x07 // Indirect Y
#define ZPX 0x08 // Zero Page X Indexed
#define ABX 0x09 // Absolute X Indexed
#define ABY 0x0A // Absolute Y Indexed
#define REL 0x0B // Relative
#define IND 0x0C // Indirect
#define ZPY 0x0D // Zero Page Y Indexed

class mcs6502 {
	public:
		mcs6502();
		~mcs6502();
	public:
		// Registers (names taken directly from datasheet don't @ me)
		uint8_t  A = 0x00;   // Accumulator
		uint8_t  Y = 0x00;   // Index Register Y
		uint8_t  X = 0x00;   // Index Register X

		uint16_t PC= 0x0000; // Program Counter
		uint8_t  S = 0x00;   // Stack Pointer

		uint16_t AB= 0x0000; // Address Bus
		uint8_t  DB= 0x00;   // Data Bus
		bool     RW= true;   // true = read, false = write

		uint8_t  P = 0x00;   // Process Status Register
		uint8_t  IR= 0x00;   // Instruction Register

	public: // Addressing Mode Functions
		// Return true if page crossed (for adding to cycles)
		bool immediate();
		bool absolute();
		bool zeropage();
		bool accumulator();
		bool implied();
		bool indirectx();
		bool indirecty();
		bool zeropagex();
		bool absolutex();
		bool absolutey();
		bool relative();
		bool indirect();
		bool zeropagey();

	public: // Opcodes
		// Each opcode takes in an int representing an addressing mode
		void ADC(uint16_t addr);	void AND(uint16_t addr);	void ASL(uint16_t addr);

		void BCC(uint16_t addr);	void BCS(uint16_t addr);	void BEQ(uint16_t addr);
		void BIT(uint16_t addr); 	void BMI(uint16_t addr);	void BNE(uint16_t addr);
		void BPL(uint16_t addr); 	void BRK(uint16_t addr);	void BVC(uint16_t addr);
		void BVS(uint16_t addr);

		void CLC(uint16_t addr);	void CLD(uint16_t addr);	void CLI(uint16_t addr);
		void CLV(uint16_t addr);	void CMP(uint16_t addr);	void CPX(uint16_t addr);
		void CPY(uint16_t addr);

		void DEC(uint16_t addr);	void DEX(uint16_t addr);	void DEY(uint16_t addr);

		void EOR(uint16_t addr);

		void INC(uint16_t addr);	void INX(uint16_t addr);	void INY(uint16_t addr);
		void JMP(uint16_t addr);	void JSR(uint16_t addr);

		void LDA(uint16_t addr);	void LDX(uint16_t addr);	void LDY(uint16_t addr);
		void LSR(uint16_t addr);

		void NOP(uint16_t addr);

		void ORA(uint16_t addr);

		void PHA(uint16_t addr);	void PHP(uint16_t addr);	void PLA(uint16_t addr);
		void PLP(uint16_t addr);

		void ROL(uint16_t addr);	void ROR(uint16_t addr);	void RTI(uint16_t addr);
		void RTS(uint16_t addr);

		void SBC(uint16_t addr);	void SEC(uint16_t addr);	void SED(uint16_t addr);
		void SEI(uint16_t addr);	void STA(uint16_t addr);	void STX(uint16_t addr);
		void STY(uint16_t addr);

		void TAX(uint16_t addr);	void TAY(uint16_t addr);	void TSX(uint16_t addr);
		void TXA(uint16_t addr);	void TXS(uint16_t addr);	void TYA(uint16_t addr);

	public:
		// Non-Hardware things (Only used in emulation)*/
		uint8_t rmcycles = 0x00; // Cycles remaining for this instruction
        
		// Two maps are used for each opcode the corresponding mnemonic and
		// addressing types are found, this makes the code simpler compared
		// to storing discrete functions for each opcode but likely slows
		// down the emulator (but who cares)
		using m = mcs6502;
		std::map<uint8_t, void (m::*)(uint16_t)> opcodemnc
		{{0x69, &m::ADC}, {0x65, &m::ADC}, {0x75, &m::ADC}, {0x6D, &m::ADC}, {0x7D, &m::ADC}, {0x79, &m::ADC}, {0x61, &m::ADC}, {0x71, &m::ADC},
		 {0x29, &m::AND}, {0x25, &m::AND}, {0x35, &m::AND}, {0x2D, &m::ADC}, {0x3D, &m::AND}, {0x39, &m::AND}, {0x21, &m::AND}, {0x31, &m::AND},
		 {0x0A, &m::ASL}, {0x06, &m::ASL}, {0x16, &m::ASL}, {0x0E, &m::ASL}, {0x1E, &m::ASL},
		 {0x90, &m::BCC},
		 {0xB0, &m::BCS},
		 {0xF0, &m::BEQ},
		 {0x24, &m::BIT}, {0x2C, &m::BIT},
		 {0x30, &m::BMI},
		 {0xD0, &m::BNE},
		 {0x10, &m::BPL},
		 {0x00, &m::BRK},
		 {0x50, &m::BVC},
		 {0x70, &m::BVS},
		 {0x18, &m::CLC},
		 {0xD8, &m::CLD},
		 {0x58, &m::CLI},
		 {0xB8, &m::CLV},
		 {0xC9, &m::CMP}, {0xC5, &m::CMP}, {0xD5, &m::CMP}, {0xCD, &m::CMP}, {0xDD, &m::CMP}, {0xD9, &m::CMP}, {0xC1, &m::CMP}, {0xD1, &m::CMP},
		 {0xE0, &m::CPX}, {0xE4, &m::CPX}, {0xEC, &m::CPX},
		 {0xC0, &m::CPY}, {0xC4, &m::CPY}, {0xCC, &m::CPY},
		 {0xC6, &m::DEC}, {0xD6, &m::DEC}, {0xCE, &m::DEC}, {0xDE, &m::DEC},
		 {0xCA, &m::DEX},
		 {0x88, &m::DEY},
		 {0x49, &m::EOR}, {0x45, &m::EOR}, {0x55, &m::EOR}, {0x4D, &m::EOR}, {0x5D, &m::EOR}, {0x59, &m::EOR}, {0x41, &m::EOR}, {0x51, &m::EOR},
		 {0xE6, &m::INC}, {0xF6, &m::INC}, {0xEE, &m::INC}, {0xFE, &m::INC},
		 {0xE8, &m::INX},
		 {0xC8, &m::INY},
		 {0x4C, &m::JMP}, {0x6C, &m::JMP},
		 {0x20, &m::JSR},
		 {0xA9, &m::LDA}, {0xA5, &m::LDA}, {0xB5, &m::LDA}, {0xAD, &m::LDA}, {0xBD, &m::LDA}, {0xB9, &m::LDA}, {0xAl, &m::LDA}, {0xB1, &m::LDA},
		 {0xA2, &m::LDX}, {0xA6, &m::LDX}, {0xB6, &m::LDX}, {0xAE, &m::LDX}, {0xBE, &m::LDX},
		 {0xA0, &m::LDY}, {0xA4, &m::LDY}, {0xB4, &m::LDY}, {0xAC, &m::LDY}, {0xBC, &m::LDY},
		 {0x4A, &m::LSR}, {0x46, &m::LSR}, {0x56, &m::LSR}, {0x4E, &m::LSR}, {0x5E, &m::LSR},
		 {0xEA, &m::NOP},
		 {0x09, &m::ORA}, {0x05, &m::ORA}, {0x15, &m::ORA}, {0x0D, &m::ORA}, {0x1D, &m::ORA}, {0x19, &m::ORA}, {0x01, &m::ORA}, {0x11, &m::ORA},
		 {0x48, &m::PHA},
		 {0x08, &m::PHP},
		 {0x68, &m::PLA},
		 {0x28, &m::PLP},
		 {0x2A, &m::ROL}, {0x26, &m::ROL}, {0x36, &m::ROL}, {0x2E, &m::ROL}, {0x3E, &m::ROL},
		 {0x6A, &m::ROR}, {0x66, &m::ROR}, {0x76, &m::ROR}, {0x6E, &m::ROR}, {0x7E, &m::ROR},
		 {0x40, &m::RTI},
		 {0x60, &m::RTS},
		 {0xE9, &m::SBC}, {0xE5, &m::SBC}, {0xF5, &m::SBC}, {0xED, &m::SBC}, {0xFD, &m::SBC}, {0xF9, &m::SBC}, {0xE1, &m::SBC}, {0xF1, &m::SBC},
		 {0x38, &m::SEC},
		 {0xF8, &m::SED},
		 {0x78, &m::SEI},
		 {0x85, &m::STA}, {0x95, &m::STA}, {0x8D, &m::STA}, {0x9D, &m::STA}, {0x99, &m::STA}, {0x81, &m::STA}, {0x91, &m::STA},
		 {0x86, &m::STX}, {0x96, &m::STX}, {0x8E, &m::STX},
		 {0x84, &m::STY}, {0x94, &m::STY}, {0x8C, &m::STY},
		 {0xAA, &m::TAX},
		 {0xA8, &m::TAY},
		 {0xBA, &m::TSX},
		 {0x8A, &m::TXA},
		 {0x9A, &m::TXS},
		 {0x98, &m::TYA}}
		std::map<uint8_t, uint8_t> opcodeadm
		{{0x69, IMM}, {0x65, ZPG}, {0x75, ZPX}, {0x6D, ABS}, {0x7D, ABX}, {0x79, ABY}, {0x61, IDX}, {0x71, IDY},
		 {0x29, IMM}, {0x25, ZPG}, {0x35, ZPX}, {0x2D, ABS}, {0x3D, ABX}, {0x39, ABY}, {0x21, IDX}, {0x31, IDY},
		 {0x0A, ACC}, {0x06, ZPG}, {0x16, ZPX}, {0x0E, ABS}, {0x1E, ABX},
		 {0x90, REL},
		 {0xB0, REL},
		 {0xF0, REL},
		 {0x24, ZPG}, {0x2C, ABS},
		 {0x30, REL},
		 {0xD0, REL},
		 {0x10, REL},
		 {0x00, IMP},
		 {0x50, REL},
		 {0x70, REL},
		 {0x18, IMP},
		 {0xD8, IMP},
		 {0x58, IMP},
		 {0xB8, IMP},
		 {0xC9, IMM}, {0xC5, ZPG}, {0xD5, ZPX}, {0xCD, ABS}, {0xDD, ABX}, {0xD9, ABY}, {0xC1, IDX}, {0xD1, IDY},
		 {0xE0, IMM}, {0xE4, ZPG}, {0xEC, ABS},
		 {0xC0, IMM}, {0xC4, ZPG}, {0xCC, ABS},
		 {0xC6, ZPG}, {0xD6, ZPX}, {0xCE, ABS}, {0xDE, ABX},
		 {0xCA, IMP},
		 {0x88, IMP},
		 {0x49, IMM}, {0x45, ZPG}, {0x55, ZPX}, {0x4D, ABS}, {0x5D, ABX}, {0x59, ABY}, {0x41, IDX}, {0x51, IDY},
		 {0xE6, ZPG}, {0xF6, ZPX}, {0xEE, ABS}, {0xFE, ABX},
		 {0xE8, IMP},
		 {0xC8, IMP},
		 {0x4C, ABS}, {0x6C, IND},
		 {0x20, ABS},
		 {0xA9, IMM}, {0xA5, ZPG}, {0xB5, ZPX}, {0xAD, ABS}, {0xBD, ABX}, {0xB9, ABY}, {0xAl, IDX}, {0xB1, IDY},
		 {0xA2, IMM}, {0xA6, ZPG}, {0xB6, ZPY}, {0xAE, ABS}, {0xBE, ABY},
		 {0xA0, IMM}, {0xA4, ZPG}, {0xB4, ZPX}, {0xAC, ABS}, {0xBC, ABX},
		 {0x4A, ACC}, {0x46, ZPG}, {0x56, ZPX}, {0x4E, ABS}, {0x5E, ABX},
		 {0xEA, IMP},
		 {0x09, IMM}, {0x05, ZPG}, {0x15, ZPX}, {0x0D, ABS}, {0x1D, ABX}, {0x19, ABY}, {0x01, IDX}, {0x11, IDY},
		 {0x48, IMP},
		 {0x08, IMP},
		 {0x68, IMP},
		 {0x28, IMP},
		 {0x2A, ACC}, {0x26, ZPG}, {0x36, ZPX}, {0x2E, ABS}, {0x3E, ABX},
		 {0x6A, ACC}, {0x66, ZPG}, {0x76, ZPX}, {0x6E, ABS}, {0x7E, ABX},
		 {0x40, IMP},
		 {0x60, IMP},
		 {0xE9, IMM}, {0xE5, ZPG}, {0xF5, ZPX}, {0xED, ABS}, {0xFD, ABX}, {0xF9, ABY}, {0xE1, IDX}, {0xF1, IDY},
		 {0x38, IMP},
		 {0xF8, IMP},
		 {0x78, IMP},
		 {0x85, ZPG}, {0x95, ZPG}, {0x8D, ABS}, {0x9D, ABX}, {0x99, ABY}, {0x81, IDX}, {0x91, IDY},
		 {0x86, ZPG}, {0x96, ZPY}, {0x8E, ABS},
		 {0x84, ZPG}, {0x94, ZPX}, {0x8C, ABS},
		 {0xAA, IMP},
		 {0xA8, IMP},
		 {0xBA, IMP},
		 {0x8A, IMP},
		 {0x9A, IMP},
		 {0x98, IMP}}
	public:
		// Interrupts & Start-up
		void clk(); // Clock In
		void irq(); // Interrupt Request
		void nmi(); // Non Maskable Interrupt
		void rst(); // Reset
		void rdy(); // Ready
		void sof(); // Set Overflow Flag
	public:
		void exec(); // Execute Instruction in Data Bus
		void read(); // Write to data bus from device on address bus
	public:
		// Accessing the Status Register
		//uint8_t getP(uint8_t bit);
		//uint8_t setP(uint8_t bit, uint8_t to); // -1 = Toggle
};

mcs6502::mcs6502() {

}

mcs6502::~mcs6502() {

}

void mcs6502::exec(){
	IR = DB;
	uint8_t adm    = opcodeadm[IR];
	uint16_t addr = 0x0000;
	switch(adm){
		case IMM:
			addr = PC++;
		case ABS:
			AB = PC;
			read();
			PC++;
			addr = DB; // Read low byte to addr

			AB = PC;
			read();
			PC++;
			addr |= (DB<<8); // Read high byte to addr
		case ZPG:
			AB = PC;
			read();
			PC++;
			addr = (DB &0x00FF);
		case IDX:



	}
	(this->*opcodemnc[opcode])(addr);
}

void mcs6502::read(){
	DB = 0xEA; // For now just write nop
}

void mcs6502::clk(){
	if(rmcycles == 0){
		AB = PC;
		read();
		exec();
		PC++;
	}

	rmcycles--;
}

void mcs6502::irq(){
	if(P & PSI == 0){ // IRQ Disable is off

	}
}

void mcs6502::ADC(uint8_t adm){
	uint16_t
	switch(adm){
		case IMM:

	}
}

uint8_t mcs6502::getP(uint8_t bit) {
	return((P & bit == 0) ? 0 : 1);
}

uint8_t mcs6502::setP(uint8_t bit, uint8_t to = -1) {
	uint8_t outP = 0x00;
	if(to == -1) { // -1 = Toggle this flag
		outP = P ^ bit;
	}else if(to){
		outP = P | bit;
	}else{
		outP = P & ~bit;
	}
	P = outP;
	return(outP); // Sometimes it's helpful to get the final flags
}
