#!/usr/bin/env python3

# Copyright (c) 2016, Thomas Keh
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    1. Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#
#    2. Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#
#    3. Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import sys
import re
import os.path
from subprocess import call
from enum import Enum

class Mode(Enum):
	none = 1
	index = 2
	subindex = 3

class Converter(object):
	
	def datatype(string,subindex=0):
		pdo_communication_parameter_record = ["0x0005","0x0007","0x0005",
			"0x0006","0x0005","0x0006","0x0005"]
		pdo_mapping_parameter_record = ["0x0005",
			"0x0007","0x0007","0x0007","0x0007","0x0007","0x0007","0x0007","0x0007",
			"0x0007","0x0007","0x0007","0x0007","0x0007","0x0007","0x0007","0x0007",
			"0x0007","0x0007","0x0007","0x0007","0x0007","0x0007","0x0007","0x0007",
			"0x0007","0x0007","0x0007","0x0007","0x0007","0x0007","0x0007","0x0007",
			"0x0007","0x0007","0x0007","0x0007","0x0007","0x0007","0x0007","0x0007",
			"0x0007","0x0007","0x0007","0x0007","0x0007","0x0007","0x0007","0x0007",
			"0x0007","0x0007","0x0007","0x0007","0x0007","0x0007","0x0007","0x0007",
			"0x0007","0x0007","0x0007","0x0007","0x0007","0x0007","0x0007","0x0007"]
		sdo_parameter_record = ["0x0005","0x0007","0x0007","0x0005"]
		identity_record = ["0x0005","0x0007","0x0007","0x0007","0x0007"]
		os_debug_record = ["0x0005","0x000A","0x0005","0x000A"]
		os_command_record = ["0x0005","0x000A","0x0005","0x000A"]
		interpolation_time_period_record = ["0x0005","0x0005","0x0002"]
		interpolation_data_configuration_record = ["0x0005","0x0007","0x0007",
			"0x0005","0x0006","0x0005","0x0005"]
		vl_velocity_acceleration_deceleration_record = ["0x0005","0x0007","0x0003"]
		types = {
			"boolean": ["0x0001"],
			"integer8": ["0x0002"],
			"integer16": ["0x0003"],
			"integer32": ["0x0004"],
			"unsigned8": ["0x0005"],
			"unsigned16": ["0x0006"],
			"unsigned32": ["0x0007"],
			"real32": ["0x0008"],
			"visible_string": ["0x0009"],
			"octet_string": ["0x000A"],
			"unicode_string": ["0x000B"],
			"time_of_day": ["0x000C"],
			"time_difference": ["0x000D"],
			"domain": ["0x000F"],
			"integer24": ["0x0010"],
			"real64": ["0x0011"],
			"integer40": ["0x0012"],
			"integer48": ["0x0013"],
			"integer56": ["0x0014"],
			"integer64": ["0x0015"],
			"unsigned24": ["0x0016"],
			"unsigned40": ["0x0018"],
			"unsigned48": ["0x0019"],
			"unsigned56": ["0x001A"],
			"unsigned64": ["0x001B"],
			"pdo_communication_parameter": pdo_communication_parameter_record,#["0x0020"],
			"pdo communication parameter": pdo_communication_parameter_record,#["0x0020"],
			"PDO communication parameter record": pdo_communication_parameter_record,#["0x0020"],
			"pdo_mapping": pdo_mapping_parameter_record,#["0x0021"],
			"pdo mapping": pdo_mapping_parameter_record,#["0x0021"],
			"PDO mapping parameter record": pdo_mapping_parameter_record,#["0x0021"],
			"sdo_parameter": sdo_parameter_record,#["0x0022"],
			"sdo parameter": sdo_parameter_record,#["0x0022"],
			"sdo parameter record": sdo_parameter_record,#["0x0022"],
			"identity": identity_record,#["0x0023"],
			"Manufacturer-specific": ["0x0040"],
			"Manufacturer specific": ["0x0040"],
			# CiA 301
			"OS debug record": os_debug_record,
			"OS command record": os_command_record,
			# CiA 402
			"Interpolation time period record (0080h)": interpolation_time_period_record,
			"Interpolation time period record": interpolation_time_period_record,
			"Interpolation data configuration record (0081h)": interpolation_data_configuration_record,
			"Interpolation data configuration record": interpolation_data_configuration_record,
			"vl velocity acceleration deceleration record (0082h)": vl_velocity_acceleration_deceleration_record,
			"vl velocity acceleration deceleration record": vl_velocity_acceleration_deceleration_record
			}
		for key,value in types.items():
			if re.match(r"^\s*" + re.escape(key) + r"\s*$",string,re.IGNORECASE):
				if len(value)<=int(str(subindex),16):
					# no record type
					return value[0]
				else:
					return value[int(str(subindex),16)]
		
		print("Unknown data type: "+string)
		return "# "+string

	def objectcode(string):
		if re.match(r"^\s*VAR\s*$",string,re.IGNORECASE):
			return "0x07"
		elif re.match(r"^\s*ARRAY\s*$",string,re.IGNORECASE):
			return "0x08"
		elif re.match(r"^\s*RECORD\s*$",string,re.IGNORECASE):
			return "0x09"
		else:
			return str

	def pdomapping(string):
		if re.match(r"^\s*No\s*$",string,re.IGNORECASE):
			return "0"
		else:
			return "1"

	def access(string):
		if re.match(r"^\s*rw\s*$",string,re.IGNORECASE):
			return "rw"
		elif re.match(r"^\s*ro\s*$",string,re.IGNORECASE):
			return "ro"
		elif re.match(r"^\s*wo\s*$",string,re.IGNORECASE):
			return "wo"
		elif re.match(r"^\s*const\s*$",string,re.IGNORECASE):
			return "const"
		else:
			print("Unknown access type: "+string+" -> using rw")
			return "rw # "+string


class Parser(object):

	def __init__(self,filein,fileout):
		self.filein = filein
		self.fileout = fileout
		self.skip = True
		self.mode = Mode.none
		self.indices = []
		self.unroll = False

	def dump(self,line):
		print("DUMP "+line)
		self.fileout.write(line+"\n")

	def parseindex(self,line):

		search = re.match(r"^\s*Index\s*([\dABCDEFabcdef]{4})\s?h$", line, re.IGNORECASE)
		search2 = re.match(r"^\s*Index\s*([\dABCDEFabcdef]{4})\s?h to ([\dABCDEFabcdef]{4})\s?h$", line, re.IGNORECASE)
		if search:
			self.indices[-1]['index'] = search.group(1)
			print("Adding index "+str(self.indices[-1]['index']))
		if search2:
			self.indices[-1]['index'] = search2.group(1)
			self.indices[-1]['indexto'] = search2.group(2)
			self.indices[-1]['indexrange'] = True
			print("Adding index "+str(self.indices[-1]['index'])+" - "+str(self.indices[-1]['indexto']))
		if search or search2:
			# TODO: same thing for parsesubindex()?
			self.done_name = False
			self.done_objectcode = False
			self.done_datatype = False
			self.done_category = False

		search = re.match(r"^\s*Name\s*(\w.*)$", line, re.IGNORECASE)
		if not self.done_name and search:
			self.indices[-1]['name'] = search.group(1)
			self.done_name = True

		search = re.match(r"^\s*Object code\s*(\w.*)$", line, re.IGNORECASE)
		if not self.done_objectcode and search:
			self.indices[-1]['objectcode'] = search.group(1)
			self.done_objectcode = True

		search = re.match(r"^\s*Data type\s*(\w.*)$", line, re.IGNORECASE)
		if not self.done_datatype and search:
			self.indices[-1]['datatype'] = search.group(1)
			self.done_datatype = True

		search = re.match(r"^\s*Category\s*(\w.*)$", line, re.IGNORECASE)
		if not self.done_category and search:
			self.indices[-1]['category'] = search.group(1)
			self.done_category = True

	def parsesubindex(self,line):

		search = re.match(r"^\s*Sub-index\s*([\dABCDEFabcdef]{2})\s?h$", line, re.IGNORECASE)
		search2 = re.match(r"^\s*Sub-index\s*([\dABCDEFabcdef]{2})\s?h to ([\dABCDEFabcdef]{2})\s?h$", line, re.IGNORECASE)
		search3 = re.match(r"^\s*Sub-index\s*(\d{1,2})$", line, re.IGNORECASE)
		search4 = re.match(r"^\s*Sub-index\s*(\d{1,2}) to (\d{1,2})$", line, re.IGNORECASE)

		if search:
			self.indices[-1]['subindices'][-1]['subindex'] = search.group(1)
			print("Adding subindex "+str(self.indices[-1]['subindices'][-1]['subindex']))
		elif search2:
			self.indices[-1]['subindices'][-1]['subindex'] = search2.group(1)
			self.indices[-1]['subindices'][-1]['subindexto'] = search2.group(2)
			self.indices[-1]['subindices'][-1]['subindexrange'] = True
			print("Adding subindex "+str(self.indices[-1]['subindices'][-1]['subindex'])+" - "+str(self.indices[-1]['subindices'][-1]['subindexto']))
		elif search3:
			self.indices[-1]['subindices'][-1]['subindex'] = str(hex(int(search3.group(1))))[2:]
			print("Adding subindex "+str(self.indices[-1]['subindices'][-1]['subindex']))
		elif search4:
			self.indices[-1]['subindices'][-1]['subindex'] = str(hex(int(search4.group(1))))[2:]
			self.indices[-1]['subindices'][-1]['subindexto'] = str(hex(int(search4.group(2))))[2:]
			self.indices[-1]['subindices'][-1]['subindexrange'] = True
			print("Adding subindex "+str(self.indices[-1]['subindices'][-1]['subindex'])+" - "+str(self.indices[-1]['subindices'][-1]['subindexto']))

		search = re.match(r"^\s*Description\s*(\w.*)$", line, re.IGNORECASE)
		if search:
			self.indices[-1]['subindices'][-1]['description'] = search.group(1)

		search = re.match(r"^\s*Data type\s*(\w.*)$", line, re.IGNORECASE)
		if search:
			self.indices[-1]['subindices'][-1]['datatype'] = search.group(1)

		search = re.match(r"^\s*Entry category\s*(\w.*)$", line, re.IGNORECASE)
		if search:
			self.indices[-1]['subindices'][-1]['entrycategory'] = search.group(1)

		search = re.match(r"^\s*Access\s*(\w.*)$", line, re.IGNORECASE)
		if search:
			self.indices[-1]['subindices'][-1]['access'] = search.group(1)

		search = re.match(r"^\s*PDO mapping\s*(\w.*)$", line, re.IGNORECASE)
		if search:
			self.indices[-1]['subindices'][-1]['pdomapping'] = search.group(1)

		search = re.match(r"^\s*Value range\s*(\w.*)$", line, re.IGNORECASE)
		if search:
			self.indices[-1]['subindices'][-1]['valuerange'] = search.group(1)

		search = re.match(r"^\s*Default value\s*(\w.*)$", line, re.IGNORECASE)
		if search:
			self.indices[-1]['subindices'][-1]['defaultvalue'] = search.group(1)

	def printvar(self,map,index,appendix,rangecomment):
		self.dump("["+str(index)+"]"+rangecomment)
		self.dump("ParameterName="+str(map['name'])+str(appendix))
		self.dump("ObjectType="+str(Converter.objectcode(map['objectcode']))+" # "+str(map['objectcode']))
		self.dump("DataType="+str(Converter.datatype(str(map['datatype'])))+" # "+str(map['datatype']))
		self.dump("# Category: "+str(map['category']))
		self.dump("AccessType="+Converter.access(str(map['subindices'][-1]['access'])))
		self.dump("DefaultValue="+str(map['subindices'][-1]['defaultvalue']))
		self.dump("PDOMapping="+Converter.pdomapping(map['subindices'][-1]['pdomapping'])+" # "+str(map['subindices'][-1]['pdomapping']))
		self.dump(" ")

	def printsubindex(self,indexmap,map,index,subindex,appendix,rangecomment):
		self.dump("["+str(index)+"sub"+str(subindex)+"]"+rangecomment)
		self.dump("ParameterName="+str(map['description'])+str(appendix))
		self.dump("ObjectType=0x007 # VAR")
		if indexmap['objectcode']=="ARRAY" and int(str(subindex),16)==0:
			# array: number of items is uint8.
			self.dump("DataType=0x0005 # UNSIGNED8")
		elif indexmap['objectcode']=="ARRAY" :
			self.dump("DataType="+str(Converter.datatype(str(indexmap['datatype']),subindex))+" # "+str(indexmap['datatype']))
		self.dump("AccessType="+Converter.access(str(map['access'])))
		self.dump("# DefaultValue="+str(map['defaultvalue']))
		self.dump("PDOMapping="+Converter.pdomapping(map['pdomapping'])+" # "+str(map['pdomapping']))
		self.dump(" ")

	def printobject(self,map,index,appendix,rangecomment):
		self.dump("["+str(index)+"]"+rangecomment)
		self.dump("ParameterName="+str(map['name'])+str(appendix))
		self.dump("ObjectType="+str(Converter.objectcode(map['objectcode']))+" # "+str(map['objectcode']))
		self.dump("# SubNumber="+str(len(map['subindices'])))
		self.dump(" ")

		for subindex in map['subindices']:
			if self.unroll and subindex['subindexrange']:

				count = 1
				localsubindex = subindex['subindex']
				while int(localsubindex,16) <= int(subindex['subindexto'],16):
					self.printsubindex(map,subindex,index,localsubindex," "+str(count),"")
					count += 1
					localsubindex = format(int(localsubindex,16)+1, '#02x')

			elif subindex['subindexrange']:
				self.printsubindex(map,subindex,index,subindex['subindex'],""," # repeat until "+str(index)+"sub"+str(subindex['subindexto']))
			else:
				if subindex['subindex']=="":
					print("WARNING subindex is empty")
				else:
					self.printsubindex(map,subindex,index,subindex['subindex'],"","")

	def printindex(self,map,index,appendix,rangecomment):
		if len(map['subindices'])==1 and not map['subindices'][-1]['subindexrange']:
			self.printvar(map,index,appendix,rangecomment)
		elif len(map['subindices'])==0:
			print("WARNING no subindices");
		else:
			self.printobject(map,index,appendix,rangecomment)

	def finish(self):

		print("Writing output to "+str(self.fileout.name))

		for index in self.indices:
			if self.unroll and index['indexrange']:

				count = 1
				localindex = index['index']
				while int(localindex,16) <= int(index['indexto'],16):
					self.printindex(index,localindex," "+str(count),"")
					count += 1
					localindex = format(int(localindex,16)+1, '#04x')
			elif index['indexrange']:
				self.printindex(index,index['index'],""," # repeat until "+str(index['indexto']))
			else:
				self.printindex(index,index['index'],"","")

	def parseline(self, line):
	
		#print("parsing line: "+str(line))
		
		if self.skip and re.match(r"^.*OBJECT DESCRIPTION$", line, re.IGNORECASE):
			self.skip = False

		if self.skip:
			return

		if re.match(r"^\s*CORRIGENDUM.*$", line):
			self.skip = True
			return

		if re.match(r"^\s*Index\s*[\dABCDEFabcdef]{4}\s?h( to [\dABCDEFabcdef]{4}\s?h)?$", line, re.IGNORECASE):
			self.indices.append({
				'index': "",
				'indexto': "",
				'indexrange': False,
				'name': "",
				'objectcode': "",
				'datatype': "",
				'category': "",
				'subindices': []})
			self.mode = Mode.index

		if re.match(r"^\s*Sub-index\s*[\dABCDEFabcdef]{1,2}\s?(h)?( to [\dABCDEFabcdef]{1,2}\s?(h)?)?$",
			line, re.IGNORECASE) or (self.mode==Mode.index and re.match(
				#r"^\s*(Access|Description|Data type|Entry category|PDO mapping|Value range|Default value)\s*.*$",
				r"^\s*ENTRY DESCRIPTION.*$",
				line, re.IGNORECASE)):
			self.indices[-1]['subindices'].append({
				'subindex': "",
				'subindexto': "",
				'subindexrange': False,
				'description': "",
				'datatype': "",
				'entrycategory': "",
				'access': "",
				'pdomapping': "",
				'valuerange': "",
				'defaultvalue': ""})
			self.mode = Mode.subindex

		if self.mode == Mode.index:
			self.parseindex(line)
		elif self.mode == Mode.subindex:
			self.parsesubindex(line)

	def parse(self):
		print("Reading input from "+str(self.filein.name))
		content = self.filein.read().splitlines()
		for line in content:
			self.parseline(line)

def help():
	print("PDF to EDS (c) 2016 Thomas Keh")
	print(" ")
	print("Usage: ./pdftoeds.py input.pdf [output.eds]")

def main():

	if (len(sys.argv)<2 or len(sys.argv)>3):
		help()
		return

	filenamein = sys.argv[1]
	basename = os.path.splitext(filenamein)[0]
	filenameout = basename+".eds"
	filenametemp = filenamein+".txt"
	if (len(sys.argv)>=3):
		filenameout = sys.argv[2]

	if os.path.isfile(filenamein):
		print("Running pdftotext -layout "+filenamein+" "+filenametemp)
		if call(["pdftotext","-layout",filenamein,filenametemp]) == 0:
			with open(filenametemp) as filein:
				with open(filenameout,'w') as fileout:
					parser = Parser(filein, fileout)
					parser.parse()
					parser.finish()
		else:
			print("pdftotext failed. Please install pdftotext")
		#call(["rm ",filenametemp])
	else:
		print("Input file not found: "+filenamein)

if __name__ == '__main__':
	main()

