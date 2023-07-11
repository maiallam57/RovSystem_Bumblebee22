
import sys; sys.path.append('/home/rov/.local/lib/python3.6/site-packages')
import subprocess
#sys.path.append('/home/rov/Desktop/underwater')
print("yesss")
#subprocess.Popen('python3 colorCamera_stream_script.py',shell=True)

while True:
    subprocess.call("python3 PilotServer.py",shell=True)
    #kill -9 $(ps -A | grep python | awk '{print $1}')
    print("end")
