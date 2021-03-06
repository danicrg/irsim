# Intelligent Robotics Simulator

## Dependencies

Ubuntu:
`apt-get install automake libreadline5-dev libstdc++6-dev libgsl0-dev libgl1-mesa-dev libglu1-mesa-dev libode-dev make g++`

Debian:
`apt-get install automake libreadline5-dev libstdc++6-4.4.-dev libgsl0-dev libgl1-mesa-dev libglu1-mesa-dev libode-dev make g++`


## Installation
```
./bootstrap.sh
./configure
make
```

## Run Experiments


`./irsim -E `

gives the list of experiments available


`./irsim -E <number>`
 
where number belongs to the interval shown in the previous command


## How to make movies

1) You need to create .ppm files (Ctrl+W when the simulator is running)

2) Once the simulator has stopped, you need to convert the .ppm to .png 
   Here I include the commands:

   a) If you want the .png files in the same dir as the .ppm files:
      ```
      for f in *.ppm; do f2=`basename "$f" ppm`png; echo "$f -> $f2"; convert "$f" "$f2"; done
      ```

   b) If you want the .png files in another directoy:
      (tape command on origin where the ppm's are)
      ```
      for f in *.ppm; do f2=`basename "$f" ppm`png; echo "$f -> $f2"; convert "$f" <ending_directory>/"$f2";done
      ```

3) Once we have the .pngfiles we need to create the movie. 
   There are different configurations depending on the movie we want to create, here I include some of them.

   a) If you want the movie in the same directory as the .png files:
      ```
      mencoder mf://*.png -mf w=800:h=600:fps=25:type=png -ovc lavc -lavcopts vcodec=mpeg4:mbd=2:trell -oac copy -o output.avi
      ```

   b) If you want the movie in another directory:	
      ```
      mencoder mf://*.png -mf w=800:h=600:fps=25:type=png -ovc lavc -lavcopts vcodec=mpeg4:mbd=2:trell -oac copy -o <ending-directory>/output.avi
      
      mencoder mf://*.png -mf w=800:h=600:fps=25:type=png -ovc copy -oac copy -o output.avi
      ```
      

4) More information can be found in:
   - http://www.mplayerhq.hu/DOCS/HTML/en/menc-feat-enc-images.html
   - http://www.tevs.eu/blog_8.html
