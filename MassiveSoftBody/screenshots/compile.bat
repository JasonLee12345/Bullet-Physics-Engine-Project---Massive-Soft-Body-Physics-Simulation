ffmpeg -framerate 30 -i frame%d.tiff -c:v libx264 -r 30 -pix_fmt yuv420p out.mp4