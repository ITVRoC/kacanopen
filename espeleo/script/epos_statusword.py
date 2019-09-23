# 00094 #define E_BIT15        0x8000      ///< bit code: position referenced to home position
# 00095 #define E_BIT14        0x4000      ///< bit code: refresh cycle of power stage
# 00096 #define E_BIT13        0x2000      ///< bit code: OpMode specific, some error
# 00097 #define E_BIT12        0x1000      ///< bit code: OpMode specific
# 00098 #define E_BIT11        0x0800      ///< bit code: NOT USED
# 00099 #define E_BIT10        0x0400      ///< bit code: Target reached
# 00100 #define E_BIT09        0x0200      ///< bit code: Remote (?)
# 00101 #define E_BIT08        0x0100      ///< bit code: offset current measured (?)
# 00102 #define E_BIT07        0x0080      ///< bit code: WARNING
# 00103 #define E_BIT06        0x0040      ///< bit code: switch on disable
# 00104 #define E_BIT05        0x0020      ///< bit code: quick stop
# 00105 #define E_BIT04        0x0010      ///< bit code: voltage enabled
# 00106 #define E_BIT03        0x0008      ///< bit code: FAULT
# 00107 #define E_BIT02        0x0004      ///< bit code: operation enable
# 00108 #define E_BIT01        0x0002      ///< bit code: switched on
# 00109 #define E_BIT00        0x0001      ///< bit code: ready to switch on          

# https://www.maxongroup.com/medias/sys_master/root/8834323283998/EPOS-Firmware-Specification-En.pdf (page 144)
word_status_code = {
	0: "Ready to switch on ",
	1: "Switched on",
	2: "Operation enable",
	3: "FAULT",
	4: "Voltage enabled (power stage on)",
	5: "Quick stop",
	6: "Switch on disable",
	7: "not used (WARNING)",
	8: "Offset current measured (?)", 
	9: "Remote (NMT Slave State Operational)",
	10: "Target reached",
	11: "NOT USED",
	12: "OpMode specific (PPM:Set-point ack, PVM:Speed, HMM:Homing attained)",
	13: "OpMode specific, some error (PPM:Following error, PVM:Not used, HMM:Homing error)",
	14: "refresh cycle of power stage",
	15: "position referenced to home position"
}

sample_statuses = [1847, 823, 5943]

for stat_word in sample_statuses:
	print("statusWord:{}".format(stat_word))
	for bit, v in sorted(word_status_code.items(), key=lambda x: x[0], reverse=False):
		if (stat_word & (1 << bit)):
			print("\t bit:{} {}".format(bit, v))
