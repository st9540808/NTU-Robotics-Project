from random import sample
from rospy import sleep
import numpy as np
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray

id_list = []
robot_is_end = 0

state = 0
def get_unflipped_cards() -> list:
  """
  Get id and position of all unflipped cards on the table

  Returns
  -------
  list
    A list with all aruco marker id of the unflipped cards on the table
  """
  filtered_list = [int(x) for x in id_list if int(x) <= 7]
  return filtered_list
  # global state
  # state += 1
  # if state <= 10:
  #   return [2, 4, 5, 7]
  # if state <= 20:
  #   return [2, 4]
  # else:
  #   return []
  return []

def robot_flip_two_card(ids):
  print(ids)
  global robot_is_end
  robot_is_end = 0
  """
  Flip two cards on the table with id in the list `ids`.
  The function needs to block until the robot complete flipping two cards.

  Parameters
  ----------
  list : ids
    cards with aruco marker id that needed to be flipped
  """
  pub_start.publish(1)
  

  while(1):
    # robot_is_end = 0
    pub_id.publish(Int32MultiArray(data = ids))
    rospy.Subscriber('move_end', Int32, is_end)
    # print('is end = ', robot_is_end)
    if(robot_is_end==1):
      print('end')
      break
  
  # sleep(220)


def is_end(end):
  global robot_is_end
  robot_is_end = end.data

mapping = {
  0:  'heart A',
  2:  'heart 2',
  4:  'heart 3',
  6:  'heart 4',
  # 8:  'heart 5',
  # 10: 'heart 6',
  # 12: 'heart 7',
  # 14: 'heart 8',
  1:  'spade A',
  3:  'spade 2',
  5:  'spade 3',
  7:  'spade 4',
  # 9:  'spade 5',
  # 11: 'spade 6',
  # 13: 'spade 7',
  # 15: 'spade 8',
}

def is_game_over():
  ids = get_unflipped_cards()
  print(ids)
  return len(ids) == 2

# class get_ID:
#   def __init__(self):
#     self.my_list = []
#     rospy.Subscriber('/aruco_id', Float32MultiArray, self.my_cb)

#   def my_cb(self, msg):
#     self.my_list.append(msg.data)

#   def get_data(self):
#     return self.my_list

def get_ID(msg):
  id = msg.data
  global id_list
  id_list = id



def wait_for_2cards_flipped(thres=50):
  times = 0
  cards_prev = get_unflipped_cards()
  cards_curr = get_unflipped_cards()
  print(cards_prev)
  while times < thres:
    if len(cards_prev) - len(cards_curr) == 2:
      times += 1
    else:
      times = 0
    cards_curr = get_unflipped_cards()
    # print(cards_curr)
    sleep(0.1) # sleep 0.1 sec and check cards on table
  
  diff = set(cards_prev) - set(cards_curr) # get set complement
  diff = list(diff)
  assert len(diff) == 2
  return diff

def wait_for_2cards_unflipped(thres=50):
  times = 0
  cards_prev = get_unflipped_cards()
  cards_curr = get_unflipped_cards()
  while times < thres:
    if len(cards_curr) - len(cards_prev) == 2:
      times += 1
    else:
      times = 0
    cards_curr = get_unflipped_cards()
    sleep(0.1) # sleep 0.1 sec and check cards on table

  diff = set(cards_curr) - set(cards_prev) # get set complement
  diff = list(diff)
  assert len(diff) == 2

def main():
  print('\a')
  print('game starting')
  turn = 0 # human turn

  while (not is_game_over()):
    if turn == 0: # human turn
      print('human turn begins')
      diff = wait_for_2cards_flipped()

      # check if card number is the same
      print(diff, mapping[diff[0]], mapping[diff[1]])
      if mapping[diff[0]][-1] == mapping[diff[1]][-1]:
        print('human turn continues for one more round')
        continue
      else:
        print('wait for human to replace the two flipped cards on the table (human turn)')
        wait_for_2cards_unflipped()
        print('human turn ends')
        turn = 1
    elif turn == 1: # robot turn
      print('robot turn begins')

      # randomly choose two cards on the table
      cards_prev = get_unflipped_cards()
      id_card_1, id_card_2 = sample(list(cards_prev), 2)
      robot_flip_two_card([id_card_1, id_card_2])

      cards_curr = get_unflipped_cards()
      assert (len(cards_prev) - len(cards_curr)) == 2

      # store difference of current cards and previous card into a list
      diff = set(cards_prev) - set(cards_curr) # get set complement
      diff = list(diff)
      assert len(diff) == 2

      # check if card numbers are the same
      if mapping[diff[0]][-1] == mapping[diff[1]][-1]:
        print('robot turn continues for one more round')
        continue
      else:
        print('wait for human to replace the two flipped cards on the table (robot turn)')
        wait_for_2cards_unflipped()
        print('robot turn ends')
        turn = 0

  print('game over')
  if turn == 0:
    print('human wins!')
    print('human wins!')
    print('human wins!')
  elif turn == 1:
    print('robot wins!')
    print('robot wins!')
    print('robot wins!')


if __name__ == '__main__':
  rospy.init_node('game')
  rospy.Subscriber('aruco_id', Float32MultiArray, get_ID)
  pub_start = rospy.Publisher('robot_start', Int32)
  pub_id = rospy.Publisher('id_talker', Int32MultiArray)
  sleep(2)
  main()
