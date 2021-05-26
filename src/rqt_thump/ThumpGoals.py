#!/usr/bin/env python
import os
from datetime import datetime
import rospy
import rospkg
import random
import re

from itertools import product
from string import join, split

from std_msgs.msg import *
from std_srvs.srv import Trigger, TriggerResponse, Empty
from diagnostic_msgs.msg import KeyValue
from rosplan_dispatch_msgs.msg import *
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
from rosplan_dispatch_msgs.msg import NonBlockingDispatchAction, NonBlockingDispatchActionGoal
from task_plan_verbalization.srv import *
from actionlib import SimpleActionClient
from PyQt5 import QtGui, QtCore
from python_qt_binding import loadUi, QT_BINDING_VERSION
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot

if QT_BINDING_VERSION.startswith('4'):
    from python_qt_binding.QtGui import QHeaderView, QIcon, QTreeWidgetItem, QListWidgetItem, QWidget
else:
    from python_qt_binding.QtWidgets import QHeaderView, QTreeWidgetItem, QListWidgetItem, QWidget
    from python_qt_binding.QtGui import QIcon

predicates = {"robot_at": "Go to", "object_at_wp": "Move object", "object_at_person": "Give object",
              "object_at_robot": "Grasp object", "person_found": "Find person", "wp_clean": "Clean"}
pred_keys = list(predicates.keys())
pred_vals = list(predicates.values())
predicates_templates = {"robot_at": "/0 goes to /1", "object_at_wp": "Move /0 to /1", "wp_clean": "Clean /0",
                        "object_at_person": "Give /0 to /1", "object_at_robot": "/1 has /0", "person_found": "Find /0"}


class PlanViewWidget(QWidget):
    _action_list = []
    _verbalization_text = ""
    _action_text = ""
    _status_list = {}
    _predicate_param_type_list = {}
    _predicate_param_label_list = {}

    # model view
    _type_list = []
    _goal_list = {}
    _fact_list = {}

    # set seed for random
    random.seed(10)

    def __init__(self, plugin=None):
        super(PlanViewWidget, self).__init__()

        self._verbalization_seed = random.random()
        self._path = rospy.get_param("rqt_path")

        # Create QWidget
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_thump'), 'resource', 'UserStudyPlugin.ui')
        loadUi(ui_file, self)

        self.setObjectName('UserStudyPluginUi')
        self._problem_generated = False
        self._plan_generated = False
        self._robot_status = "Ready to plan"
        self.goal_refresh = True
        self.goal_dict = {}

        with open(self._path, "a") as save_file:
            save_file.write("----------------- \n")
            save_file.write("Experiment started at: " + str(datetime.now().strftime('%H: %M: %S %p')) + "\n")

        # populate goal combo boxes
        try:
            rospy.wait_for_service('rosplan_knowledge_base/domain/predicates', 5.0)
            predicates_client = rospy.ServiceProxy('rosplan_knowledge_base/domain/predicates',
                                                   GetDomainAttributeService)
            resp = predicates_client()
            for pred in resp.items:
                if pred.name in predicates.keys():
                    self.goalNameComboBox.addItem(predicates[pred.name])
                    param_list = []
                    label_list = []
                    for param in pred.typed_parameters:
                        param_list.append(param.value)
                        label_list.append(param.key)
                    self._predicate_param_type_list[pred.name] = param_list
                    self._predicate_param_label_list[pred.name] = label_list
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        self._handle_goal_name_changed(0)

        # Verbalization overview
        try:
            rospy.wait_for_service('rosplan_narrator/narrate_current_plan', 1000)
            verbalization_client = rospy.ServiceProxy('rosplan_narrator/narrate_plan', NarratePlan)
            resp = verbalization_client('', -1, self._verbalization_seed)
            self.verbalizationView.insertPlainText(resp.narration)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        # connect components
        self.planButton.clicked[bool].connect(self._handle_plan_clicked)
        self.dispatchButton.clicked[bool].connect(self._handle_dispatch_clicked)
        self.stopButton.clicked[bool].connect(self._handle_stop_clicked)
        self.removeGoalButton.clicked[bool].connect(self._handle_remove_goal_clicked)
        self.addGoalButton.clicked[bool].connect(self._handle_add_goal_clicked)
        self.newGoalButton.clicked[bool].connect(self._handle_new_goal_clicked)
        self.goalNameComboBox.currentIndexChanged[int].connect(self._handle_goal_name_changed)

        self.photo.setPixmap(QtGui.QPixmap("/home/yani/thump_ws/src/rqt_thump/resource/map700.png"))

        self.initUI()

        self._plugin = plugin

        # init and start update timers
        self._timer_refresh_goals = QTimer(self)
        self._timer_refresh_goals.timeout.connect(self.refresh_model)
        # self._timer_refresh_verbalization = QTimer(self)
        # self._timer_refresh_goals.timeout.connect(self.refresh_verbalization)

        # The rosplan service calls for executing the problem
        self._problem_gen = rospy.ServiceProxy("/rosplan_problem_interface/problem_generation_server", Empty)
        self._planner = rospy.ServiceProxy("/rosplan_planner_interface/planning_server", Empty)
        self._parse_plan = rospy.ServiceProxy("/rosplan_parsing_interface/parse_plan", Empty)
        self._dispatch_plan = SimpleActionClient("/rosplan_plan_dispatcher/dispatch_plan_action",
                                                 NonBlockingDispatchAction)
        self._cancel_dispatch = rospy.ServiceProxy("/rosplan_plan_dispatcher/cancel_dispatch", Empty)
        self._plan_narration = rospy.ServiceProxy("/rosplan_narrator/narrate_plan", NarratePlan)
        self._predicate_narration = rospy.ServiceProxy("/rosplan_narrator/narrate_predicate", NarratePredicate)
        self._narration_param = rospy.ServiceProxy("/rosplan_narrator/set_params", SetVerbalizationParams)

        # Create subscribers to prob_generation and planner
        rospy.Subscriber("/rosplan_problem_interface/problem_instance", String, self.problem_callback)
        rospy.Subscriber("/rosplan_planner_interface/planner_output", String, self.planner_callback)
        rospy.Subscriber("/rosplan_plan_dispatcher/action_dispatch", ActionDispatch, self.dispatcher_callback)

        self.refresh_model()

    """
    Start the UI refresh timer
    """

    def start(self):
        self._timer_refresh_goals.start(1000)
        # self._timer_refresh_verbalization.start(1000)

    """
    Initialise the buttons and labels on the UI
    """

    def initUI(self):
        self.planButton.setEnabled(True)
        self.dispatchButton.setEnabled(False)
        self.stopButton.setEnabled(False)
        self.removeGoalButton.setEnabled(True)
        self.addGoalButton.setEnabled(False)
        self.newGoalButton.setEnabled(True)
        self.statusLabel.setText(self._robot_status)

    def save_file(self, message):
        with open(self._path, "a") as save_file:
            save_file.write(str(datetime.now().strftime('%H: %M: %S %p')) + ": " + message + '\n')

    """
    Updating view
    """

    def refresh_model(self):
        # Status
        self.statusLabel.setText(self._robot_status)

        # goals
        rospy.wait_for_service('rosplan_knowledge_base/state/goals')
        selected_list = []
        for item in self.goalView.selectedItems():
            selected_list.append(item.text())
        try:
            if self.goal_refresh:
                self.goal_refresh = False
                goals_client = rospy.ServiceProxy('rosplan_knowledge_base/state/goals', GetAttributeService)
                resp = goals_client('')
                self.goalView.clear()
                self._goal_list.clear()
                for goal in resp.attributes:
                    item = QListWidgetItem(self.goalView)
                    goalText = '(' + goal.attribute_name
                    for keyval in goal.values:
                        goalText = goalText + ' ' + keyval.value
                    goalText = goalText + ')'
                    resp_pred = self._predicate_narration(goalText)
                    item.setText(resp_pred.verbalization)

                    self.goal_dict[resp_pred.verbalization] = goalText
                    self._goal_list[goalText] = goal
                    if goalText in selected_list:
                        item.setSelected(True)

            # self.refresh_verbalization()
            r = re.compile(r'\*\s*(.*)\n', re.MULTILINE)
            self._verbalization_text = r.sub(r'<b>\1</b>\n', self._verbalization_text)
            self.verbalizationView.setHtml(self._verbalization_text.replace('\n', '<br>'))

            action = ""
            for line in self._action_text.split("\n"):
                if '*' in line:
                    action = line[2:]
                    break
            self.actionView.setHtml(action)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    """
    Refresh the verbalization section
    """

    # def refresh_verbalization(self):


    """
    called when the plan button is clicked; sends a planning request
    """

    def _handle_plan_clicked(self, checked):
        self.save_file("Planning clicked")
        self._robot_status = "Planning..."

        self._problem_gen.call()
        while not self._problem_generated:
            rospy.sleep(1)

        self._problem_generated = False

        try:
            self._planner.call()

            while not self._plan_generated:
                rospy.sleep(1)
            self._plan_generated = False

            self._parse_plan.call()
            rospy.sleep(1.0)
            self.dispatcher_callback(ActionDispatch(action_id=-1))

            self.dispatchButton.setEnabled(True)
            # self._timer_refresh_verbalization.setInterval(15000)
            self._timer_refresh_goals.setInterval(100000)
            self.refresh_model()

            self._robot_status = "Finished planning, ready to execute"
            self.statusLabel.setText(self._robot_status)
        except rospy.ServiceException as e:
            self.save_file("Planning failed")
            self._robot_status = "Couldn't find a plan! Plan Unsolvable."
            rospy.logerr(rospy.get_name() + ": Service call failed: %s" % e)

    """
    Method used when dispatching starts, updating the UI and sending the actions
    """

    def _handle_dispatch_clicked(self):
        self.save_file("Execution started.")
        self._robot_status = "Executing plan"
        self.planButton.setEnabled(False)
        self.removeGoalButton.setEnabled(False)
        self.addGoalButton.setEnabled(False)
        self.stopButton.setEnabled(True)

        self._timer_refresh_goals.setInterval(1000)

        self._dispatch_plan.send_goal(NonBlockingDispatchGoal())

    """
    Stop execution when stop clicked, re-initilising the UI.
    """

    def _handle_stop_clicked(self):
        self.save_file("Execution stopped.")
        self._robot_status = "Stopped execution"
        self._cancel_dispatch.call()

        self.initUI()

    """
    Problem callback, useful for knowing when the problem is generated
    """

    def problem_callback(self, data):
        self._problem_generated = True

    """
    Planner callback, useful for knowing when the planning ends
    """

    def planner_callback(self, data):
        self._plan_generated = True

    """
    Dispatcher callback, used for the verbalization of the plan execution, 
    and for keeping track of the execution. 
    Reinitialise the UI at the end
    """

    def dispatcher_callback(self, data):
        try:
            resp_param = self._narration_param(3, 1, 1, 2, "1", 2, 4)
            resp = self._plan_narration('', data.action_id, self._verbalization_seed)
            self._verbalization_text = resp.narration

            resp_param = self._narration_param(3, 1, 1, 2, "1", 3, 1)
            resp = self._plan_narration('', data.action_id, self._verbalization_seed)
            self._action_text = resp.narration
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        result = self._dispatch_plan.get_result()
        print result
        if not result is None:
            if self._dispatch_plan.get_result().success:
                self._robot_status = "Execution success, goals removed."
                all_items = []
                for index in range(self.goalView.count()):
                    all_items.append(self.goalView.item(index))
                self._handle_remove_button_clicked(KnowledgeUpdateServiceRequest.REMOVE_GOAL, all_items,
                                                   self._goal_list)
                # self.refresh_model()
                self.initUI()
            else:
                self._robot_status = "Execution failed..."
                self.initUI()

    """
    handle changing selected goal or fact predicate name
    """

    def _handle_predicate_name_change(self, pred_name, combo):
        combo.clear()
        rospy.wait_for_service('rosplan_knowledge_base/state/instances')
        parameters = []
        if pred_name not in self._predicate_param_type_list:
            return None
        for param_type in self._predicate_param_type_list[pred_name]:
            try:
                predicates_client = rospy.ServiceProxy('rosplan_knowledge_base/state/instances', GetInstanceService)
                resp = predicates_client(param_type)
                parameters.append(resp.instances)
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e
        for element in product(*parameters):
            pred = predicates_templates[pred_name]
            for i, p in enumerate(element):
                pred = pred.replace('/' + str(i), p)
            if not "unknown" in pred:
                combo.addItem(pred)

    """
    Additional method for creating a new goal
    """

    def _handle_goal_name_changed(self, index):
        self._handle_predicate_name_change(pred_keys[pred_vals.index(self.goalNameComboBox.itemText(index))],
                                           self.goalComboBox)

    """
    called when the add goal/fact button is clicked
    """

    def _handle_add_button_clicked(self, update_type, pred_name, combo):
        rospy.wait_for_service('rosplan_knowledge_base/update')
        template = predicates_templates[pred_name]
        try:
            update_client = rospy.ServiceProxy('rosplan_knowledge_base/update', KnowledgeUpdateService)
            knowledge = KnowledgeItem()
            knowledge.knowledge_type = KnowledgeItem.FACT
            knowledge.attribute_name = pred_name
            index = 0

            x = re.findall('/\d', template)
            ts = template.split(' ')
            ss = combo.currentText().split(' ')
            text = ''
            for tt in x:
                text += ss[ts.index(tt)] + ' '

            for param in split(text):
                pair = KeyValue()
                pair.key = (self._predicate_param_label_list[knowledge.attribute_name])[index]
                index = index + 1
                pair.value = param
                knowledge.values.append(pair)
            resp = update_client(update_type, knowledge)
            self.save_file("Added (" + pred_name + " " + combo.currentText() + ")")
        except rospy.ServiceException, e:
            self.save_file("Add-button failed!")
            print "Service call failed: %s" % e

    def _handle_add_goal_clicked(self, data):
        self._robot_status = "Added new goal. Ready to plan."
        self.initUI()
        self._handle_add_button_clicked(KnowledgeUpdateServiceRequest.ADD_GOAL,
                                        pred_keys[pred_vals.index(self.goalNameComboBox.currentText())],
                                        self.goalComboBox)
        self.goal_refresh = True
        self.refresh_model()

    """
    Invoked when the new goal btn is clicked, making the add goal btn available
    """

    def _handle_new_goal_clicked(self, data):
        self._robot_status = "Select predicate with parameters for new goal."
        self.addGoalButton.setEnabled(True)

    """
    called when the remove goal button is clicked
    """

    def _handle_remove_button_clicked(self, updateType, removeNameList, removeMsgList):
        rospy.wait_for_service('rosplan_knowledge_base/update')
        for item in removeNameList:
            try:
                update_client = rospy.ServiceProxy('rosplan_knowledge_base/update', KnowledgeUpdateService)
                resp = update_client(updateType, removeMsgList[self.goal_dict[item.text()]])

                self.save_file("Removed predicate " + self.goal_dict[item.text()])
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e

    """
    Invoked when the remove goal btn is clicked, removing the goals from the KB and reinitialising the UI
    """

    def _handle_remove_goal_clicked(self, checked):
        self._robot_status = "Removed goal(s). Ready to plan."
        self.initUI()
        self._handle_remove_button_clicked(KnowledgeUpdateServiceRequest.REMOVE_GOAL, self.goalView.selectedItems(),
                                           self._goal_list)
        self.goal_refresh = True
        self.refresh_model()

    """
    Qt methods
    """

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
