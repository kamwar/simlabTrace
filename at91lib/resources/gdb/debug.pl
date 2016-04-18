# Perl script to easily launch AT91 debug sessions.

use File::Basename;

# List of supported boards
my @boards = ("at91sam7se-ek",
             "at91sam9260-ek",
             "at91sam9261-ek",
             "at91sam9263-ek",
             "at91sam9rl-ek",
             "at91sam9xe-ek",
             "at91sam9g20-ek",
             "at91sam9m10-ek",
             "at91cap9-dk",
             "at91cap9-stk"
             );

# Check that an argument has been provided
if (!@ARGV[0]) {

   print("Usage: " . basename($0) . " <elf-file>\n");
   exit(1);
}

# Parse file name
my $file = @ARGV[0];
my $script = "";
my $gdb = dirname($0);

# Check #2: this must be an elf file
if ($file !~ m/.*.elf/i) {

   print(".elf file expected.\n");
   exit(2);
}

# Check #1: 'sdram' or 'ddram' or 'bcram' token in filename
if (($file =~ m/.*sdram.*/i) or ($file =~ m/.*ddram.*/i) or ($file =~ m/.*bcram.*/i) or ($file =~ m/.*sam9.*/i) or ($file =~ m/.*cap9.*/i) ) {

   # Find board basename
   foreach $board (@boards) {

      if (index($file, $board) != -1) {

         $script = "$gdb\\$board";
      }
   }

   # Add -ek-mck or -ek-sdram depending on need
   if ($file =~ m/.*sdram.*/i) {

      $script .= "-sdram.gdb";
   }
   elsif ($file =~ m/.*ddram.*/i) {

      $script .= "-ddram.gdb";
   }
   elsif ($file =~ m/.*bcram.*/i) {

      $script .= "-bcram.gdb";
   }   
   else {

      $script .= "-sram.gdb";
   }
}

# Create command file to define "reset" command
open(CMD, ">cmd.gdb") or die("Could not create command file:\n$!");
print(CMD "define reset\n");
print(CMD "    target remote localhost:2331\n");
print(CMD "    monitor reset\n");
if ($script) {

   print(CMD "    source $script\n");
}
print(CMD "    load\n");
print(CMD "end");
close(CMD);

# Launch GDB
$pid = fork();
if ($pid == 0) {

   exec("arm-none-eabi-gdb -x cmd.gdb -ex \"reset\" $file");
}
else {

   $SIG{INT} = 'IGNORE';
   $res = waitpid($pid, 0);
}
print("Done\n");
unlink("cmd.gdb");
