from IPython.core.display_functions import display
import ipywidgets as widgets
from ipywidgets import Output, Button, HBox

robot_one = [('Select', None), ('PR2', "pr2"), ('Tiago', "tiago")]
robot_two = [('Select', None), ('PR2', "pr2"), ('Tiago', "tiago")]
environments = [('Select', None), ('Apartment', "apartment-small.urdf"),
                ('Kitchen (Unavailable)', None)]  # ('Kitchen', "kitchen-small.urdf")

selected_context = None
selected_environment = None
selected_location = None


def setup_task_object_widgets():
    robot_one_dropdown = widgets.Dropdown(options=robot_one, description='First Robot:')
    robot_two_dropdown = widgets.Dropdown(options=robot_two, description='Second Robot:')
    environment_dropdown = widgets.Dropdown(options=environments, description='Environment:')

    robot_one_dropdown.observe(lambda change: update_globals(context=change['new']), names='value')
    robot_two_dropdown.observe(lambda change: update_globals(location=change['new']), names='value')
    environment_dropdown.observe(lambda change: update_globals(environment=change['new']), names='value')

    display(HBox([robot_one_dropdown, robot_two_dropdown, environment_dropdown]))


def update_globals(context=None, environment=None, location=None):
    global selected_context, selected_environment, selected_location
    if context is not None:
        selected_context = context
    if environment is not None:
        selected_environment = environment
    if location is not None:
        selected_location = location


def robot_execute(func):
    global selected_context, selected_environment, selected_location
    with output:
        output.clear_output()
        func(selected_location, selected_context, selected_environment)


def start_demo(func):
    global output
    output = Output()
    setup_task_object_widgets()
    execute_button = Button(description="Execute Task")
    # Use a lambda function to defer the call to `robot_execute`
    # In this lambda function, lambda x: robot_execute(func),
    # x represents the button click event (which we don't use here),
    # and robot_execute(func) is the function call you want to happen when the button is clicked.
    # This way, robot_execute will only be executed when the button is clicked, not when start_demo is called.
    execute_button.on_click(lambda x: robot_execute(func))
    display(execute_button, output)
