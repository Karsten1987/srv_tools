#!/usr/bin/env python
"""
Copyright (c) 2012,
Systems, Robotics and Vision Group
University of the Balearican Islands
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Systems, Robotics and Vision Group, University of 
      the Balearican Islands nor the names of its contributors may be used to 
      endorse or promote products derived from this software without specific 
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""


PKG = 'bag_tools' # this package name

import roslib; roslib.load_manifest(PKG)
import rospy
import tempfile
import subprocess
import glob
import shutil

def create_video(tmp_dir, inbag, topic, output, fps):
  rospy.loginfo('Using {} as working directory.'.format(tmp_dir))
  rospy.loginfo('Extracting images...')

  cmd = ["rosrun", "bag_tools", "extract_images" , tmp_dir, "jpg", topic, inbag]
  rospy.loginfo('    {}'.format(' '.join(cmd)))
  subprocess.call(cmd)

  rospy.loginfo("Renaming...")
  images = glob.glob(tmp_dir + '/*.jpg')
  images.sort()
  print len(images)
  print tmp_dir
  i = 1
  for image in images:
    #shutil.move(image, tmp_dir + '/img-' + str(i) + '.jpg')
    i = i + 1

  rospy.loginfo('Creating video...')
  cmd = ["mencoder", "-nosound", "mf://"+str(tmp_dir)+"/*.jpg", "-mf", "w=320:h=240:type=jpg:fps="+str(fps), "-ovc", "lavc", "-lavcopts", "vcodec=mpeg4:vbitrate=2400", "-o", output]
  rospy.loginfo('    {}'.format(' '.join(cmd)))
  print '    {}'.format(' '.join(cmd))
  subprocess.call(cmd)


if __name__ == "__main__":
  import argparse
  parser = argparse.ArgumentParser(
      description=
        'Creates a video from sensor_msgs/Image messages from a bagfile. '
        'This script uses the extract_images binary to extract color images '
        'from bagfiles and calls ffmpeg afterwards to combine them together '
        'to form a video. Note that ffmpeg must be installed on your system.')
  parser.add_argument('topic', help='topic of the images to use')
  parser.add_argument('--output', help='name of the output video. Note that the file ending defines the codec to use.', default='video.mp4')
  parser.add_argument('--fps', help='frames per second in the output video, as long as codec supports this', type=int, default=10)
  parser.add_argument('inbag', help='input bagfile(s)', nargs='+')
  parser.add_argument('-nodelete', action='store_true')
  args = parser.parse_args()

  import os
  from os import path
  for raw_inbag in args.inbag:
    filename, ext = os.path.splitext( os.path.split( raw_inbag )[1] )

    tmp_dir = tempfile.mkdtemp()
    try:
      create_video(tmp_dir, raw_inbag, args.topic, "videos/"+str(filename)+".mp4", args.fps)
    except Exception, e:
      import traceback
      traceback.print_exc()
    if not args.nodelete:
      shutil.rmtree( tmp_dir )
