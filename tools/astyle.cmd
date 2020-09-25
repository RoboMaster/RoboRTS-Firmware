@echo off
set AstylePath=%~dp0
%AstylePath%\AStyle.exe *.c *.h --recursive  --style=bsd --indent=spaces=4 --convert-tabs ^
--align-pointer=name --pad-oper --pad-header --indent-preproc-block --indent-col1-comments --unpad-paren --add-braces