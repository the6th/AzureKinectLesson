 set PATH=%PATH%;C:\Program Files\VCG\MeshLab

 del /Q x64\Release\out2\

  for %%V in (x64/Release/out/*.ply) do (
    meshlabserver -i x64/Release/out/%%V   -o x64/Release/out2/%%~nV.ply -m vc -s plyAlign.mlx
  )


  pause

  