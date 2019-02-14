# Navigator  
本项目提供unity脚本用以导出navmesh数据为recastnavigation可直接使用的tile cache文件
 
# 用法  
1）先把exporter_for_unity里的文件复制到你的unity项目的随便一个Editor目录下；  
2）烘培好NavMesh后通过菜单NavMeshExporter->ExportToNavBin把navmesh数据导出成bin文件，可通过RecastDemo里Load来看看是否正常；  

# Usage 
This project provides unity scripts to export navmesh data into tile cache files that can be used directly by recastnavigation, and export some API for lua.   
1)copy files in folder exporter_for_unity to any 'Editor' folder in your unity project;   
2)after baked navmesh, then click NavMeshExporter->ExportToNavBin menu to export navmesh data to tile cache bin file, you can load the file in RecastDemo.  

# TODO
)导出lua接口

# Note
)if you want to rebuild exporter_for_unity\NavigatorForUnity.dll, you can run recastnavigation\RecastDemo\Build\vs2015\recastnavigation.sln in windows system, and rebuild the NavigatorForUnity project.  
)if you get a error with "fatal error C1083: 无法打开包括文件: “SDL.h”: No such file or directory", you should grab the latest SDL2 development library release from here and unzip it recastnavigation\RecastDemo\Contrib. Rename the SDL folder such that the path recastnavigation\RecastDemo\Contrib\SDL\lib\x86 is valid. 