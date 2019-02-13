# Navigator  
本项目提供unity脚本用以导出navmesh数据为recastnavigation可直接使用的tile cache文件
 
# 用法  
1）先把exporter_for_unity里的文件复制到你的unity项目的随便一个Editor目录下；  
2）烘培好NavMesh后通过菜单NavMeshExporter->ExportToNavBin把navmesh数据导出成bin文件，可通过RecastDemo里Load来看看是否正常；  

convert navmesh data that export from unity to tile cache, use for recastnavigation in server side, and export some API for lua.  
# Usage 
)copy files in folder exporter_for_unity to any 'Editor' folder in your unity project;   
)after baked navmesh, then click NavMeshExporter->ExportToNavBin menu to export navmesh data to tile cache bin file, you can load the file in RecastDemo.  

# TODO
)导出lua接口