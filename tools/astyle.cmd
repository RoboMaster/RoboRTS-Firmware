@echo off
set AstylePath=%~dp0
%AstylePath%\AStyle.exe *.c *.h --recursive  --style=bsd --indent=spaces=4 --convert-tabs --align-pointer=type --pad-oper --indent-preproc-block --indent-col1-comments --unpad-paren --add-braces