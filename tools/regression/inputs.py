"""Read-only MID360/RGB connection contracts and optional decoded-pixel ledger."""
import hashlib,json,struct
from pathlib import Path
TOPICS={'B':['/livox/mid360/lidar','/livox/mid360/imu','/camera/color/image_raw/compressed']}
EXPECTED={TOPICS['B'][0]:('livox_ros_driver2/CustomMsg','e4d6829bdfe657cb6c21a746c86b21a6'),TOPICS['B'][1]:('sensor_msgs/Imu','6a62c6daae103f4ff57a132d6f95cec2'),TOPICS['B'][2]:('sensor_msgs/CompressedImage','8f7a12909da2c9d3332d540a0977563f')}
def header(data):
 seq,sec,nsec,size=struct.unpack_from('<IIII',data,0)
 return seq,sec*1000000000+nsec,data[16:16+size].decode(),16+size

def inspect_bag(path,mode='B'):
 import rosbag
 with rosbag.Bag(str(path),'r',skip_index=True) as bag:
  counts={}
  for chunk in bag._chunks:
   for cid,n in chunk.connection_counts.items():counts[cid]=counts.get(cid,0)+n
  connections=[dict(topic=c.topic,type=c.header['type'].decode() if isinstance(c.header['type'],bytes) else c.header['type'],md5=c.md5sum,count=counts.get(c.id,0)) for c in bag._connections.values()]
  metadata=dict(start=bag.get_start_time(),end=bag.get_end_time(),duration=bag.get_end_time()-bag.get_start_time(),message_count=bag.get_message_count())
 errors=[]
 for topic in TOPICS[mode]:
  found=[c for c in connections if c['topic']==topic]
  if not found or not sum(c['count'] for c in found):errors.append('Missing/nonempty input: '+topic)
  if any((c['type'],c['md5'])!=EXPECTED[topic] for c in found):errors.append('Type/MD5 mismatch: '+topic)
 if errors:raise ValueError('; '.join(errors))
 return dict(required_topics=TOPICS[mode],connections=connections,metadata=metadata,scope='Recorded type/count contract only; parameter-to-sequence applicability must be separately established.')

def image_contract(bag_path):
 import rosbag
 from cv_bridge import CvBridge
 bridge=CvBridge();images={}
 with rosbag.Bag(str(bag_path),'r') as bag:
  for _,msg,_ in bag.read_messages(topics=[TOPICS['B'][2]]):
   img=bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='rgb8')
   images[str(msg.header.stamp.to_nsec())]={'rgb':hashlib.sha256(img.tobytes()).hexdigest()}
 return {'image_hashes':images,'source_bag':str(bag_path),'encoding':'rgb8','generation':'cv_bridge compressed decoder; checked again against actual standard-plugin deliveries'}
