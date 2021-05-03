#!/usr/bin/perl
##
##  printenv -- demo CGI program which just prints its environment
##

print "Content-type: text/plain\n\n";
print "<html><head><title>test cgi environment</title></head><body>\n";
foreach(sort keys %ENV) {
    print "$_ : $ENV{\"$_\"}<br>\n";
}
print "</body></html>";
