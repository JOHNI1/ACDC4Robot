import adsk.core
import os
from ...lib import fusion360utils as futil
from ... import config
from . import acdc4robot
from . import constants
app = adsk.core.Application.get()
ui = app.userInterface


# File to store the last modification code
LAST_MODIFICATION_CODE_FILE = os.path.join(os.path.dirname(__file__), 'last_modification_code.txt')


# Default example code
DEFAULT_EXAMPLE_CODE = (
    """#modify robot_ele (class Element from xml.etree.ElementTree). Its the api for you!
for _ in range(10):
    futil.log('hello')

"""
)

# Function to load the last modification code from the file
def load_last_modification_code():
    if os.path.exists(LAST_MODIFICATION_CODE_FILE):
        with open(LAST_MODIFICATION_CODE_FILE, 'r') as file:
            return file.read()
    return DEFAULT_EXAMPLE_CODE  # Return default if file doesn't exist

# Function to save the modification code to the file
def save_modification_code(code):
    with open(LAST_MODIFICATION_CODE_FILE, 'w') as file:
        file.write(code)


# TODO *** Specify the command identity information. ***
CMD_ID = f'{config.COMPANY_NAME}_{config.ADDIN_NAME}_ACDC4Robot'
CMD_NAME = 'ACDC4Robot'
CMD_Description = 'Export Autodesk Fusion design model to robot description format'

# Specify that the command will be promoted to the panel.
IS_PROMOTED = True

# TODO *** Define the location where the command button will be created. ***
# This is done by specifying the workspace, the tab, and the panel, and the 
# command it will be inserted beside. Not providing the command to position it
# will insert it at the end.
WORKSPACE_ID = 'FusionSolidEnvironment'
PANEL_ID = 'SolidScriptsAddinsPanel'
COMMAND_BESIDE_ID = 'ScriptsManagerCommand'

# Resource location for command icons, here we assume a sub folder in this directory named "resources".
ICON_FOLDER = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'resources', '')

# Local list of event handlers used to maintain a reference so
# they are not released and garbage collected.
local_handlers = []


# Executed when add-in is run.
def start():
    # Create a command Definition.
    cmd_def = ui.commandDefinitions.addButtonDefinition(CMD_ID, CMD_NAME, CMD_Description, ICON_FOLDER)

    # Define an event handler for the command created event. It will be called when the button is clicked.
    futil.add_handler(cmd_def.commandCreated, command_created)

    # ******** Add a button into the UI so the user can run the command. ********
    # Get the target workspace the button will be created in.
    workspace = ui.workspaces.itemById(WORKSPACE_ID)

    # Get the panel the button will be created in.
    panel = workspace.toolbarPanels.itemById(PANEL_ID)

    # Create the button command control in the UI after the specified existing command.
    control = panel.controls.addCommand(cmd_def, COMMAND_BESIDE_ID, False)

    # Specify if the command is promoted to the main toolbar. 
    control.isPromoted = IS_PROMOTED


# Executed when add-in is stopped.
def stop():
    # Get the various UI elements for this command
    workspace = ui.workspaces.itemById(WORKSPACE_ID)
    panel = workspace.toolbarPanels.itemById(PANEL_ID)
    command_control = panel.controls.itemById(CMD_ID)
    command_definition = ui.commandDefinitions.itemById(CMD_ID)

    # Delete the button command control
    if command_control:
        command_control.deleteMe()

    # Delete the command definition
    if command_definition:
        command_definition.deleteMe()

# Function that is called when a user clicks the corresponding button in the UI.
# This defines the contents of the command dialog and connects to the command related events.
def command_created(args: adsk.core.CommandCreatedEventArgs):
    # General logging for debug.
    futil.log(f'{CMD_NAME} Command Created Event')

    # create command inputs for model.config
    inputs = args.command.commandInputs

    # Add a description field to instruct the user.
    instruction_text = (
        "Welcome to fork of ACDC4Robot for building advanced robots!\n"
        "This tool allows you to export your Autodesk Fusion 360 design model to a xacro file for Gazebo physics simulator.\n"
        "Except of creating the links and joints from the model, you can add your own modification code to the link or joints or even extra links and joints using python exec function.\n"
        "The path to the mesh folder of the stl files should be configured by the xacro:property mesh_folder_path."
    )
    inputs.addTextBoxCommandInput("instruction_text", "Instructions", instruction_text, len(instruction_text.splitlines()), True)

	
    # Load the last modification code or use the default
    last_modification_code = load_last_modification_code()
    inputs.addTextBoxCommandInput("modification_code", "Modification code:", last_modification_code, 40, False)


    # TODO Connect to the events that are needed by this command.
    futil.add_handler(args.command.execute, command_execute, local_handlers=local_handlers)
    futil.add_handler(args.command.inputChanged, command_input_changed, local_handlers=local_handlers)
    futil.add_handler(args.command.destroy, command_destroy, local_handlers=local_handlers)

# This event handler is called when the user clicks the OK button in the command dialog or 
# is immediately called after the created event not command inputs were created for the dialog.
def command_execute(args: adsk.core.CommandEventArgs):
    # General logging for debug.
    futil.log(f'{CMD_NAME} Command Execute Event')


    # Get the command inputs value
    inputs = args.command.commandInputs
    modification_code: adsk.core.TextBoxCommandInput = inputs.itemById("modification_code")
	
    # Save the modification code to the file
    save_modification_code(modification_code.text)

    # Set it in constants for further processing
    constants.MODIFICATION_CODE = modification_code.text
    

    futil.log(f"Received modification code:\n {constants.MODIFICATION_CODE}")

    # Run the core function
    acdc4robot.run()



# # Function that is called when a user clicks the corresponding button in the UI.
# # This defines the contents of the command dialog and connects to the command related events.
# def command_created(args: adsk.core.CommandCreatedEventArgs):
#     # General logging for debug.
#     futil.log(f'{CMD_NAME} Command Created Event')

#     # create command inputs for model.config
#     inputs = args.command.commandInputs

#     # Add a description field to instruct the user.
#     instruction_text = (
#         "Welcome to fork of ACDC4Robot for building drones!\n"
#         "This tool allows you to export your Autodesk Fusion 360 design model to a xacro file for Gazebo physics simulator.\n"
#         "Except of creating the links and joints, it also has a feature of adding necessary plugins\n"
#         "The path to the mesh folder of the stl files should be configured by the xacro:property mesh_folder_path.\n"
#     )
#     inputs.addTextBoxCommandInput("instruction_text", "Instructions", instruction_text, 4, True)

#     # Add a text box input for IMU coordinates
#     imu_input_text = (
#         "imu coordinate x:\n"
#         "imu coordinate y:\n"
#         "imu coordinate z:"
#     )
#     inputs.addTextBoxCommandInput("imu_coordinates", "IMU Coordinates", imu_input_text, 3, False)

#     # TODO Connect to the events that are needed by this command.
#     futil.add_handler(args.command.execute, command_execute, local_handlers=local_handlers)
#     futil.add_handler(args.command.inputChanged, command_input_changed, local_handlers=local_handlers)
#     futil.add_handler(args.command.destroy, command_destroy, local_handlers=local_handlers)

# # This event handler is called when the user clicks the OK button in the command dialog or 
# # is immediately called after the created event not command inputs were created for the dialog.
# def command_execute(args: adsk.core.CommandEventArgs):
#     # General logging for debug.
#     futil.log(f'{CMD_NAME} Command Execute Event')

#     # Get the command inputs value
#     inputs = args.command.commandInputs
#     imu_input: adsk.core.TextBoxCommandInput = inputs.itemById("imu_coordinates")

#     # Log or process the IMU coordinates text
#     imu_text = imu_input.text
#     futil.log(f"Received IMU Coordinates:\n{imu_text}")

#     # TODO Process the IMU coordinates as needed

#     acdc4robot.run()


# # Function that is called when a user clicks the corresponding button in the UI.
# # This defines the contents of the command dialog and connects to the command related events.
# def command_created(args: adsk.core.CommandCreatedEventArgs):
#     # General logging for debug.
#     futil.log(f'{CMD_NAME} Command Created Event')

#     # create command inputs for model.config
#     inputs = args.command.commandInputs

#     # Add a description field to instruct the user.
#     instruction_text = (
#         "Welcome to fork of ACDC4Robot for building drones!\n"
#         "This tool allows you to export your Autodesk Fusion 360 design model to a xacro file for Gazebo physics simulator.\n"
#         "Except of creating the links and joints, it also has a feature of adding neccessary plugins\n"
#         "The path to the mesh folder of the stl files should be configured by the xacro:property mesh_folder_path.\n"
#     )
#     inputs.addTextBoxCommandInput("instruction_text", "Instructions", instruction_text, 4, True)


#     # # create a drop down command input to choose robot description format
#     # rdf_input = inputs.addDropDownCommandInput("robot_description_format", "Robot Description Format", adsk.core.DropDownStyles.LabeledIconDropDownStyle)
#     # rdf_items = rdf_input.listItems
#     # rdf_items.add("None", True)
#     # rdf_items.add("URDF", False)
#     # rdf_items.add("SDFormat", False)
#     # rdf_items.add("MJCF", False)

#     # # create a drop down command input to choose simulation environment
#     # sim_env_input = inputs.addDropDownCommandInput("simulation_env", "Simulation Environment", adsk.core.DropDownStyles.LabeledIconDropDownStyle)
#     # sim_env_items = sim_env_input.listItems
#     # sim_env_items.add("None", True)
#     # sim_env_items.add("Gazebo", False)
#     # sim_env_items.add("PyBullet", False)
#     # sim_env_items.add("MuJoCo", False)
#     # sim_env_input.isVisible = False

#     # # create string value input for sdf info
#     # sdf_author_input = inputs.addStringValueInput("SDF_Author_name", "Author Name", "ACDC4Robot")
#     # sdf_author_input.isVisible = False
#     # sdf_description_input = inputs.addTextBoxCommandInput("SDF_Description", "Description", "Description about the robot model", 3, False)
#     # sdf_description_input.isVisible = False

#     # # https://help.autodesk.com/view/fusion360/ENU/?contextId=CommandInputs
#     # inputs = args.command.commandInputs

#     # # TODO Define the dialog for your command by adding different inputs to the command.

#     # # Create a simple text box input.
#     # inputs.addTextBoxCommandInput('text_box', 'Some Text', 'Enter some text.', 1, False)

#     # # Create a value input field and set the default using 1 unit of the default length unit.
#     # defaultLengthUnits = app.activeProduct.unitsManager.defaultLengthUnits
#     # default_value = adsk.core.ValueInput.createByString('1')
#     # inputs.addValueInput('value_input', 'Some Value', defaultLengthUnits, default_value)

#     # TODO Connect to the events that are needed by this command.
#     futil.add_handler(args.command.execute, command_execute, local_handlers=local_handlers)
#     futil.add_handler(args.command.inputChanged, command_input_changed, local_handlers=local_handlers)
#     # futil.add_handler(args.command.executePreview, command_preview, local_handlers=local_handlers)
#     # futil.add_handler(args.command.validateInputs, command_validate_input, local_handlers=local_handlers)
#     futil.add_handler(args.command.destroy, command_destroy, local_handlers=local_handlers)


# # This event handler is called when the user clicks the OK button in the command dialog or 
# # is immediately called after the created event not command inputs were created for the dialog.
# def command_execute(args: adsk.core.CommandEventArgs):
#     # General logging for debug.
#     futil.log(f'{CMD_NAME} Command Execute Event')

#     # # get the command inputs value
#     # inputs = args.command.commandInputs
#     # sdf_input: adsk.core.DropDownCommandInput = inputs.itemById("robot_description_format")
#     # sim_env_input: adsk.core.DropDownCommandInput = inputs.itemById("simulation_env")
#     # name_input: adsk.core.StringValueCommandInput = inputs.itemById("SDF_Author_name")
#     # text_input: adsk.core.TextBoxCommandInput = inputs.itemById("SDF_Description")
#     # constants.set_rdf(sdf_input.selectedItem.name)
#     # constants.set_sim_env(sim_env_input.selectedItem.name)
#     # constants.set_author_name(name_input.value)
#     # constants.set_model_description(text_input.text)

#     acdc4robot.run()
#     # TODO ******************************** Your code here ********************************

#     # Get a reference to your command's inputs.
#     # inputs = args.command.commandInputs
#     # text_box: adsk.core.TextBoxCommandInput = inputs.itemById('text_box')
#     # value_input: adsk.core.ValueCommandInput = inputs.itemById('value_input')

#     # # Do something interesting
#     # text = text_box.text
#     # expression = value_input.expression
#     # msg = f'Your text: {text}<br>Your value: {expression}'
#     # ui.messageBox(msg)


# This event handler is called when the command needs to compute a new preview in the graphics window.
def command_preview(args: adsk.core.CommandEventArgs):
    # General logging for debug.
    futil.log(f'{CMD_NAME} Command Preview Event')
    inputs = args.command.commandInputs


# This event handler is called when the user changes anything in the command dialog
# allowing you to modify values of other inputs based on that change.
def command_input_changed(args: adsk.core.InputChangedEventArgs):
    changed_input = args.input
    command = changed_input.parentCommand
    inputs = command.commandInputs
    rdf: adsk.core.DropDownCommandInput = inputs.itemById("robot_description_format")
    sim_env: adsk.core.DropDownCommandInput = inputs.itemById("simulation_env")
    sim_env_items = sim_env.listItems
    sdf_author = inputs.itemById("SDF_Author_name")
    sdf_description = inputs.itemById("SDF_Description")
    # inputs = args.inputs

    sim_env.isVisible = False
    # if changed_input.id == "robot_description_format":
    #     if changed_input.selectedItem.name == "None":
    #         sim_env.isVisible = False
            
    #     elif changed_input.selectedItem.name == "URDF":
    #         sim_env.isVisible = True
            
    #     elif changed_input.selectedItem.name == "SDFormat":
    #         sim_env.isVisible = True
        
    #     elif changed_input.selectedItem.name == "MJCF":
    #         sim_env.isVisible = True
            

    # if (rdf.selectedItem.name == "SDFormat") and (sim_env.selectedItem.name == "Gazebo"):
    #     sdf_author.isVisible = True
    #     sdf_description.isVisible = True
    # else:
    #     sdf_author.isVisible = False
    #     sdf_description.isVisible = False

    # General logging for debug.
    futil.log(f'{CMD_NAME} Input Changed Event fired from a change to {changed_input.id}')


# This event handler is called when the user interacts with any of the inputs in the dialog
# which allows you to verify that all of the inputs are valid and enables the OK button.
def command_validate_input(args: adsk.core.ValidateInputsEventArgs):
    # General logging for debug.
    futil.log(f'{CMD_NAME} Validate Input Event')

    inputs = args.inputs
    
    # Verify the validity of the input values. This controls if the OK button is enabled or not.
    valueInput = inputs.itemById('value_input')
    if valueInput.value >= 0:
        args.areInputsValid = True
    else:
        args.areInputsValid = False
        

# This event handler is called when the command terminates.
def command_destroy(args: adsk.core.CommandEventArgs):
    # General logging for debug.
    futil.log(f'{CMD_NAME} Command Destroy Event')

    global local_handlers
    local_handlers = []
