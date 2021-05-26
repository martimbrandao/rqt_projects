import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from .ThumpGoals import PlanViewWidget



class UserStudyPlugin(Plugin):

    def __init__(self, context):
        super(UserStudyPlugin, self).__init__(context)
        self.setObjectName('UserStudyPlugin')

        # Create QWidget
        self._widget = PlanViewWidget(self)
        # Get path to UI file which should be in the "resource" folder of this package
        # ui_file = os.path.join(rospkg.RosPack().get_path('rqt_thump'), 'resource', 'UserStudyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        # loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        # self._widget.setObjectName('UserStudyPluginUi')

        self._widget.start()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()