This package uses UFTP to send files to a given IP address. The IP address, transport rate in kbit and the directory to
transmit can be specified as parameters in a launch file.

The ros nodes for the riegl scanner create on start up a new directory with the current date and time as name.
Therefore, the last directory in alphabetical order in the above-mentioned directory is selected and transported.

Use the command `uftpd -d -I <receiver IP address> -D <target directory>` to receive files. 