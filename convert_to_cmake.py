from tkinter import *
from tkinter.filedialog import askdirectory
from tkinter.filedialog import askopenfilename
import os
import sys
from xml.dom.minidom import parse
import xml.dom.minidom
def selectPath():
    global project_path
    path_ = askopenfilename()
    project_path.set(path_)

def selectCmakeSavePath():
    global cmake_save_path
    path_ = askdirectory()
    cmake_save_path.set(path_)


def convert(project_path,project_root):
    
    (path,project_file_name) = os.path.split(project_path)
    (project_name,notuse) = os.path.splitext(project_file_name)

    path = path.strip('/')	
    
    print("Keil工程文件所在目录:%s"%(path))
    print("Keil工程文件名:%s"%(project_file_name))
    print("工程根目录：%s"%project_root)

    if project_path.find(project_root) is -1:
        print("工程根目录错误，请选择一个工程文件的父级目录！")
        return

    print("开始转换....")	
    
    project_file_rela_dir = project_path[len(project_root) + 1:] 


    DOMTree = xml.dom.minidom.parse(path+'/'+project_file_name)
    collection = DOMTree.documentElement
    Groups = collection.getElementsByTagName("FilePath")
    Include_Item = collection.getElementsByTagName("Cads")[0].getElementsByTagName("IncludePath")[0]

    Include_OriStr = Include_Item.childNodes[0].data

    Include_OriStr = Include_OriStr.replace(" ",'').replace("..", "").replace("\\",'/').replace(";/"," \n").replace("/",'',1)

    print("\n\n头文件目录:\n%s"%Include_OriStr)
    pathList = {}

    cmake_template_buff = open('./cmake_template/CMakeLists.txt',encoding = 'utf-8').read()
    # print('makefile is%s'%Src_makefile_buff)

    print("\n\n源码文件:\n")

    p_path = os.path.dirname(os.path.dirname(project_file_rela_dir))

    src_files = ""

    for group in Groups:
        filePath = group.childNodes[0].data        

        (file_dir,file_name) = os.path.split(filePath)

        if(file_dir == ''):
            file_dir = p_path
        else:
            file_dir = file_dir.replace("..",p_path)
            file_dir = file_dir.replace("\\",'/')

        (shotname,extension) = os.path.splitext(file_name)
        if(extension == '.h'):
            continue
        
        print(file_dir + file_name)
        src_files +=(file_dir +"/"+ file_name + "\n").replace("/","",1)
        
    out_cmakelist = open(project_root + "/CMakeLists.txt",'w+',encoding = 'utf-8')
    out_cmakelist.write(cmake_template_buff.replace("###PROJECT_NAME###",project_name)\
                                          .replace("###SRC_FILE###",src_files)\
                                          .replace('###INC_DIR###',Include_OriStr))

    #print(pathList[the_path])
    print("转换成功！")

def main():
    global project_path
    global cmake_save_path
    root = Tk()

    project_path = StringVar()

    cmake_save_path = StringVar()

    # Label(root,text = "CMake路径:").grid(row = 0, column = 0)
    # Entry(root, textvariable = path).grid(row = 0, column = 1)
    # Button(root, text = "打开", command = selectPath).grid(row = 0, column = 2)

    # Label(root,text = "GCC工具链路径:").grid(row = 1, column = 0)
    # Entry(root, textvariable = path).grid(row = 1, column = 1)
    # Button(root, text = "打开", command = selectPath).grid(row = 1, column = 2)

    Label(root,text = "Keil工程文件:").grid(row = 0, column = 0)
    Entry(root, textvariable = project_path).grid(row = 0, column = 1)
    Button(root, text = "打开", command = selectPath).grid(row = 0, column = 2)

    
    Label(root,text = "工程根目录:").grid(row = 1, column = 0)
    Entry(root, textvariable = cmake_save_path).grid(row = 1, column = 1)
    Button(root, text = "打开", command = selectCmakeSavePath).grid(row = 1, column = 2)


    Button(root, text = "开始转换", command = lambda:convert(project_path.get(),cmake_save_path.get())).grid(row = 2, column = 0)

    root.mainloop()


if __name__ == "__main__":
    main()

