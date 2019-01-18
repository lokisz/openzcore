# file: srec2hdlmem.pl
# designer: lokisz
# date: 2011.04.16
# version: 0.11
# Description:
#	Perl script to generate verilog memory model from Motorola S-record memory format.
# To do:
#	Turn on "use strict;";
#	Generate full HDL model, according to software program ELF/S-record;
#	Display ELF/S-record software program information;
#		/record_numbers/memsize/start_address/end_address


#!/usr/bin/perl -w
#use strict;

#############################################
# Motorola S-record memory format description
#
# +-------------------//------------------//-----------------------+
# | type | count | address  |            data           | checksum |
# +-------------------//------------------//-----------------------+
#
# type -- A char[2] field. These characters describe the type of record (S0, S1, S2, S3, S5, S7, S8, or S9).
# count -- A char[2] field. These characters when paired and interpreted as a hexadecimal value, display the count of remaining character pairs in the record.
# address -- A char[4,6,or8] field. These characters grouped and interpreted as a hexadecimal value, display the address at which the data field is to be loaded into memory. The length of the field depends on the number of bytes necessary to hold the address. A 2-byte address uses 4 characters, a 3-byte address uses 6 characters, and a 4-byte address uses 8 characters.
# data -- A char [0-64] field. These characters when paired and interpreted as hexadecimal values represent the memory loadable data or descriptive information.
# checksum -- A char[2] field. These characters when paired and interpreted as a hexadecimal value display the least significant byte of the ones complement of the sum of the byte values represented by the pairs of characters making up the count, the address, and the data fields.

######
# type
#
# S0 This type of record is the header record for each block of S-records. The data field may contain any descriptive information identifying the following block of S-records. (It is commonly ``HDR'' on many systems.) The address field is normally zero.
# S1 A record containing data and the 2-byte address at which the data is to reside. 
# S2 A record containing data and the 3-byte address at which the data is to reside.
# S3 A record containing data and the 4-byte address at which the data is to reside. 
# S5 A record containing the number of S1, S2 and S3 records transmitted in a particular block. The count appears in the two-byte address field. There is no data field.
# S6 A record containing the number of S1, S2 and S3 records transmitted in a particular block. The count appears in the three-byte address field. There is no data field.
# S7 A termination record for a block of S3 records. The address field may contain the 4-byte address of the instruction to which control is passed. There is no data field.
# S8 A termination record for a block of S2 records. The address field may optionally contain the 3-byte address of the instruction to which control is passed. There is no data field.
# S9 A termination record for a block of S1 records. The address field may optionally contain the 2-byte address of the instruction to which control is passed. If not specified, the first entry point specification encountered in the object module input will be used. There is no data field.

$DebugOn = 0;
print "\n---> Program running...\n";

# Open s-record file:test.txt
open(SRECFILE,"go.srec") || die "$!";
print "--->File open OK!\n" if($DebugOn eq 1);

#Creat hdl model file:ramdata
#if ramdata exit, delete old one
if(open(TEST,"ramdata")){
	print "---> Deleting old ramdata\n";
	system("rm ramdata");
	}
close(TEST);
open(HDLFILE,">>ramdata") || die "$!";

@Srecords=<SRECFILE>;
$numofrecord=$#Srecords+1;
print "--->The S-record number in test.txt is $numofrecord\n" if($DebugOn eq 1);

# function to transfer a record to address-bytes HDL memory format
sub OneRecord2HDL {
	my ($RecordString)=@_;
	chomp $RecordString;
	@OneRecord = split (//,$RecordString);
	print "\n The record is ",@OneRecord  if($DebugOn eq 1);
	my ($numtmp,$numdata);
	$numtmp=$#OneRecord-6;
	print "\n The number of ASIC char is ", $#OneRecord if($DebugOn eq 1);
	#print "\n", $numtmp, "\n" if($DebugOn eq 1);
	my ($tag,$type,$recordlength,$baseaddress,@data,$checksum);
	
	# to add
	$tag = $OneRecord[0];
	if($tag ne 'S'){
		print "\n !!! This is not a valid S-record file.\n";
		die;
	}
		
	$type = $OneRecord[1];
	$recordlenth = join('',$OneRecord[2],$OneRecord[3]);
	#print $recordlenth;
	if ($type eq 0){}
	elsif ($type eq 1){
		$baseaddress = oct(join('',0,x,$OneRecord[4],$OneRecord[5],$OneRecord[6],$OneRecord[7]));
		$numdata = ($numtmp - 4)/2;
		print "\nThe number of data is ",$numdata if($DebugOn eq 1);
		my @data_add;
		my @datacontent;
		for($i=0;$i<$numdata;$i++){
			$data_add[$i] = $baseaddress + $i;
			$datacontent[$i] = join('',$OneRecord[12+2*$i],$OneRecord[12+1+2*$i]);
			print HDLFILE sprintf "@%04x\n",$data_add[$i];
			print HDLFILE $datacontent[$i],"\n";
		}
	}
	elsif ($type eq 2){
		$baseaddress = oct(join('',0,x,$OneRecord[4],$OneRecord[5],$OneRecord[6],$OneRecord[7],$OneRecord[8],$OneRecord[9]));
		$numdata = ($numtmp - 6)/2;
		print "\nThe number of data is ",$numdata if($DebugOn eq 1);
		my @data_add;
		my @datacontent;
		for($i=0;$i<$numdata;$i++){
			$data_add[$i] = $baseaddress + $i;
			$datacontent[$i] = join('',$OneRecord[12+2*$i],$OneRecord[12+1+2*$i]);
			print HDLFILE sprintf "@%06x\n",$data_add[$i];
			print HDLFILE $datacontent[$i],"\n";
		}
	}
	elsif ($type eq 3){
		$baseaddress = oct(join('',0,x,$OneRecord[4],$OneRecord[5],$OneRecord[6],$OneRecord[7],$OneRecord[8],$OneRecord[9],$OneRecord[10],$OneRecord[11]));
		$numdata = ($numtmp - 8)/2;
		print "\nThe number of data is ",$numdata if($DebugOn eq 1);
		my @data_add;
		my @datacontent;
		for($i=0;$i<$numdata;$i=$i+4){
			$data_add[$i] = $baseaddress + $i;
			$datacontent[$i] = join('',$OneRecord[12+2*$i],$OneRecord[12+1+2*$i], $OneRecord[12+2+2*$i],$OneRecord[12+3+2*$i],$OneRecord[12+4+2*$i],$OneRecord[12+5+2*$i],$OneRecord[12+6+2*$i],$OneRecord[12+7+2*$i]);  
			#print HDLFILE sprintf "@%08x\n",$data_add[$i];
			print HDLFILE $datacontent[$i],"\n";
		}
		#$checksum = join('',$OneRecord[($#OneRecord)-1],$OneRecord[$#OneRecord];
#		print sprintf "\n@%8x",@data_add;
#		print "\n",$datacontent;


	}
	elsif ($type eq 5){}
	elsif ($type eq 6){}
	elsif ($type eq 7){}
	elsif ($type eq 8){}
	elsif ($type eq 9){}
	else{
		print "\n !!! This is not a valid S-record file.\n";
		print "\n The type of record is:",$type;
		die;
	}
}

foreach $onerecord (@Srecords){
	OneRecord2HDL($onerecord);
}

print "\n---> Verilog memory model(ramdata) is OK!\n";

close(SRECFILE);
close(HDLFILE);
