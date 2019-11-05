#!/usr/bin/env python

import json
import rclpy
import numpy as np
import baldor as br
from rclpy.clock import ROSClock
from geometry_msgs.msg import TransformStamped
from handeye_tf_service.srv import HandeyeTF
from ament_index_python.resources import get_resource

# Rqt widgets
from rqt_gui_py.plugin import Plugin
from python_qt_binding import QtCore
from python_qt_binding.QtGui import QIcon, QImage, QPixmap, QStandardItem, \
                                                QIntValidator, QStandardItemModel
from python_qt_binding.QtWidgets import (QComboBox, QAction, QToolBar, QStatusBar,
                                                 QLineEdit, QWidget, QVBoxLayout,
                                                       QLabel, QTextEdit, QFrame,
                                                          QHBoxLayout, QTreeView)

class bcolors:
  HEADER = '\033[95m'
  OKBLUE = '\033[94m'
  OKGREEN = '\033[92m'
  WARNING = '\033[93m'
  FAIL = '\033[91m'
  ENDC = '\033[0m'
  BOLD = '\033[1m'
  UNDERLINE = '\033[4m'

def save_samples_to_file(samples, file_name='dataset.json', pkg='handeye_dashboard'):
  """
  Saving transform samples to a disc file
  Parameters
  -------------
  samples: list
    A list of transforms.
  file_name: string
    The destination file.
  Returns
  --------
  Success: bool
    Execution status
  """
  success = False
  samples_list = []
  for sample in samples:
    samples_list +=[[sample[0].tolist(), sample[1].tolist()]]

  _, path_pkg = get_resource('packages', pkg)

  import json
  # If the file name exists, write a JSON string into the file.
  if file_name != None:
    filename = '/tmp/' + file_name
    # Writing JSON data
    with open(filename, 'w') as f:
      json.dump(samples_list, f)
      success = True

  return success

class HandEyeCalibration(Plugin):
  PLUGIN_TITLE = ' Intel OTC Robotics: Hand-Eye Calibration'
  def __init__(self, context):
    super(HandEyeCalibration, self).__init__(context)
    self.context = context
    self.node = context.node
    self.widget = QWidget()
    self.widget.setObjectName(self.PLUGIN_TITLE)
    self.widget.setWindowTitle(self.PLUGIN_TITLE)

    # Data
    self.Tsamples = []

    # Toolbar
    _, path_pkg = get_resource('packages', 'handeye_dashboard')
    print("{}".format(path_pkg))

    self.snapshot_action = QAction(QIcon.fromTheme('camera-photo'),
                                                 'Take a snapshot', self.widget)
    path = path_pkg + '/share/handeye_dashboard/images/capture.png'
    self.calibrate_action = QAction(QIcon(QPixmap.fromImage(QImage(path))),
                                         'Get the camera/robot transform', self.widget)
    self.clear_action = QAction(QIcon.fromTheme('edit-clear'),
                              'Clear the record data.', self.widget)
    path = path_pkg + '/share/handeye_dashboard/images/UR5.png'
    self.execut_action = QAction(QIcon(QPixmap.fromImage(QImage(path))),
                                          'EStart the publishing the TF.', self.widget)
    self.toolbar = QToolBar()
    self.toolbar.addAction(self.snapshot_action)
    self.toolbar.addAction(self.calibrate_action)
    self.toolbar.addAction(self.clear_action)
    self.toolbar.addAction(self.execut_action)

    # Toolbar0
    self.l0 = QLabel(self.widget)
    self.l0.setText("Camera-Mount-Type: ")
    self.l0.setFixedWidth(150)
    self.l0.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
    self.combobox = QComboBox(self.widget)
    self.combobox.addItem('attached on robot')
    self.combobox.addItem('fixed beside robot')
    self.toolbar0 = QToolBar()
    self.toolbar0.addWidget(self.l0)
    self.toolbar0.addWidget(self.combobox)

    # Toolbar1
    self.l1 = QLabel(self.widget)
    self.l1.setText("Camera-Frame: ")
    self.l1.setFixedWidth(150)
    self.l1.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
    self.camera_frame = QLineEdit(self.widget)
    self.camera_frame.setText("camera_link")
    self.toolbar1 = QToolBar()
    self.toolbar1.addWidget(self.l1)
    self.toolbar1.addWidget(self.camera_frame)

    # Toolbar2
    self.l2 = QLabel(self.widget)
    self.l2.setText("Object-Frame: ")
    self.l2.setFixedWidth(150)
    self.l2.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
    self.object_frame = QLineEdit(self.widget)
    self.object_frame.setText("calib_board")
    self.toolbar2 = QToolBar()
    self.toolbar2.addWidget(self.l2)
    self.toolbar2.addWidget(self.object_frame)


    # Toolbar3
    self.l3 = QLabel(self.widget)
    self.l3.setText("Robot-Base-Frame: ")
    self.l3.setFixedWidth(150)
    self.l3.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
    self.base_frame = QLineEdit(self.widget)
    self.base_frame.setText("base")
    self.toolbar3 = QToolBar()
    self.toolbar3.addWidget(self.l3)
    self.toolbar3.addWidget(self.base_frame)

    # Toolbar4
    self.l4 = QLabel(self.widget)
    self.l4.setText("End-Effector-Frame: ")
    self.l4.setFixedWidth(150)
    self.l4.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
    self.endeffector_frame = QLineEdit(self.widget)
    self.endeffector_frame.setText("tool0")
    self.toolbar4 = QToolBar()
    self.toolbar4.addWidget(self.l4)
    self.toolbar4.addWidget(self.endeffector_frame)

    # Toolbar5
    self.l5 = QLabel(self.widget)
    self.l5.setText("Sample-Number: ")
    self.l5.setFixedWidth(150)
    self.l5.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
    self.le5 = QLineEdit(self.widget)
    self.le5.setValidator(QIntValidator())
    self.le5.setText('10')
    self.le5.setReadOnly(True)
    self.toolbar5 = QToolBar()
    self.toolbar5.addWidget(self.l5)
    self.toolbar5.addWidget(self.le5)

    # TreeView
    self.treeview = QTreeView()
    self.treeview.setAlternatingRowColors(True)
    self.model = QStandardItemModel(self.treeview)
    self.treeview.setModel(self.model)
    self.treeview.setHeaderHidden(True)

    # TextEdit
    self.textedit = QTextEdit(self.widget)
    self.textedit.setReadOnly(True)

    # Layout
    self.layout = QVBoxLayout()
    self.layout.addWidget(self.toolbar0)
    self.layout.addWidget(self.toolbar1)
    self.layout.addWidget(self.toolbar2)
    self.layout.addWidget(self.toolbar3)
    self.layout.addWidget(self.toolbar4)
    self.layout.addWidget(self.toolbar5)
    self.layout.addWidget(self.toolbar)
    self.layoutH = QHBoxLayout()
    self.layoutH.addWidget(self.treeview)
    self.layoutH.addWidget(self.textedit)
    self.layout.addLayout(self.layoutH)
    self.widget.setLayout(self.layout)
    # Add the widget to the user interface
    if context.serial_number() > 1:
      self.widget.setWindowTitle(self.widget.windowTitle() +
                                            (' (%d)' % context.serial_number()))
    context.add_widget(self.widget)
    # Make the connections
    self.snapshot_action.triggered.connect(self.take_snapshot)
    self.calibrate_action.triggered.connect(self.calibration)
    self.clear_action.triggered.connect(self.clear)
    self.execut_action.triggered.connect(self.execution)

    # Package path
    self.path_pkg = path_pkg

    # Set up TF
    self.cli = self.node.create_client(HandeyeTF, 'handeye_tf_service')
    while not self.cli.wait_for_service(timeout_sec=1.0):
        self.node.get_logger().info('service not available, waiting again...')
    self.req = HandeyeTF.Request()

  def clear(self):
    # >>> Clear the recorded samples
    self.textedit.append('Clearing the recorded data ...')
    self.textedit.clear()
    self.Tsamples = []
    self.model.clear()

  def get_tf_transform(self, frame_id, child_frame_id):
    self.req.transform.header.frame_id = frame_id
    self.req.transform.child_frame_id = child_frame_id
    self.req.publish.data = False

    future = self.cli.call_async(self.req)
    rclpy.spin_until_future_complete(self.node, future)

    transform = TransformStamped()

    try:
      result = future.result()
    except Exception as e:
      self.node.get_logger().info('Service call failed %r' % (e,))
    else:
      transform = result.tf_lookup_result

    return transform

  def publish_tf_transform(self, transform_to_publish):
    self.req.publish.data = True
    self.req.transform = transform_to_publish

    future = self.cli.call_async(self.req)
    rclpy.spin_until_future_complete(self.node, future)

    try:
      future.result()
    except Exception as e:
      self.node.get_logger().info('Service call failed %r' % (e,))
    else:
      self.node.get_logger().info('Send the camera-robot transform :\n\tfrom `{}` to `{}`.'.
                                                            format(self.req.transform.header.frame_id,
                                                                   self.req.transform.child_frame_id))
    
  def take_snapshot(self):
    # >>> Take the snapshot
    self.textedit.append('Taking snapshot ...')

    # Get the transform from `tool0` to `base_link`
    T = self.get_tf_transform(self.base_frame.text(), self.endeffector_frame.text())
    bTe = np.zeros((4,4))
    q = [T.transform.rotation.w, T.transform.rotation.x, T.transform.rotation.y,
          T.transform.rotation.z]
    bTe = br.quaternion.to_transform(q)
    bTe[:3, 3] = np.array([T.transform.translation.x, T.transform.translation.y,
                                                      T.transform.translation.z])
    self.textedit.append('Lookup transform: from `{}` to `{}`.'.
                          format(self.base_frame.text(), self.endeffector_frame.text()))
    self.node.get_logger().info(bcolors.OKGREEN + 'bTe:' + bcolors.ENDC + '\n{}'.format(bTe))

    # Get the transform from `calib_board` to `camera_link`
    T = self.get_tf_transform(self.camera_frame.text(), self.object_frame.text())
    cTo = np.zeros((4,4))
    q = [T.transform.rotation.w, T.transform.rotation.x, T.transform.rotation.y,
          T.transform.rotation.z]
    cTo = br.quaternion.to_transform(q)
    cTo[:3, 3] = np.array([T.transform.translation.x, T.transform.translation.y,
                                                      T.transform.translation.z])
    self.textedit.append('Lookup transform: from `{}` to `{}`.'.
                          format(self.camera_frame.text(), self.object_frame.text()))
    self.node.get_logger().info(bcolors.OKGREEN + 'cTo:' + bcolors.ENDC + '\n{}'.format(cTo))
  
    parent = QStandardItem('Snapshot {}'.format(len(self.Tsamples)))
    child_1 = QStandardItem('bTe:\n{}\n{}\n{}\n{}'.format(bTe[0, :], bTe[1, :], bTe[2, :], bTe[3, :]))
    child_2 = QStandardItem('cTo:\n{}\n{}\n{}\n{}'.format(cTo[0, :], cTo[1, :], cTo[2, :], cTo[3, :]))
    parent.appendRow(child_1)
    parent.appendRow(child_2)
    self.model.appendRow(parent)
    self.Tsamples.append((bTe, cTo))
    self.le5.setText(str(len(self.Tsamples)))

  def calibration(self):
    # >>> Compute the calibration 
    self.textedit.append('Making the calibration ...')
    if len(self.Tsamples) == 0:
      self.textedit.append('No transform recorded, please take snapshots.')
      return
    # save samples to `dataset.json` file
    save_samples_to_file(self.Tsamples)
    import handeye
    if self.combobox.currentIndex() == 0:
      solver_cri = handeye.calibrator.HandEyeCalibrator(setup='Moving')
    if self.combobox.currentIndex() == 1:
      solver_cri = handeye.calibrator.HandEyeCalibrator(setup='Fixed')
    for sample in self.Tsamples:
      solver_cri.add_sample(sample[0], sample[1])
    try:
      bTc = solver_cri.solve(method=handeye.solver.Daniilidis1999)
      # save the calibration result to 'camera-robot.json' file
      file_output = '/tmp/' + 'camera-robot.json'
      with open(file_output, 'w') as f:
        json.dump(bTc.tolist(), f)
    except Exception:
      self.textedit.append("Failed to solve the hand-eye calibration.")

  def execution(self):
    # >>> Publish the camera-robot transform
    self.textedit.append('Publishing the camera TF ...')
    file_input = '/tmp/' + 'camera-robot.json'
    with open(file_input, 'r') as f:
      datastore = json.load(f)

    to_frame = self.camera_frame.text()
    if self.combobox.currentIndex() == 0:
      from_frame = self.endeffector_frame.text()
    if self.combobox.currentIndex() == 1:
      from_frame = self.base_frame.text()

    bTc = np.array(datastore)
    static_transformStamped = TransformStamped()
    static_transformStamped.header.stamp = ROSClock().now().to_msg()
    static_transformStamped.header.frame_id = from_frame
    static_transformStamped.child_frame_id = to_frame

    static_transformStamped.transform.translation.x = bTc[0,3]
    static_transformStamped.transform.translation.y = bTc[1,3]
    static_transformStamped.transform.translation.z = bTc[2,3]

    q = br.transform.to_quaternion(bTc)
    static_transformStamped.transform.rotation.x = q[1]
    static_transformStamped.transform.rotation.y = q[2]
    static_transformStamped.transform.rotation.z = q[3]
    static_transformStamped.transform.rotation.w = q[0]

    self.publish_tf_transform(static_transformStamped)

    output_string = "camera-robot pose:\n"
    output_string += "  Translation: [{}, {}, {}]\n".format(bTc[0,3], bTc[1,3], bTc[2,3])
    output_string += "  Rotation: in Quaternion [{}, {}, {}, {}]".format(q[0], q[1], q[2], q[3])
    file_path = '/tmp/' + 'camera-robot.txt'
    with open(file_path, 'w') as f:
      f.write(output_string)

  def shutdown_plugin(self):
    """
    Unregister subscribers when the plugin shutdown
    """
    pass

  def save_settings(self, plugin_settings, instance_settings):
    # Nothing to be done here
    pass

  def restore_settings(self, plugin_settings, instance_settings):
    # Nothing to be done here
    pass
