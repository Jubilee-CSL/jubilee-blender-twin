import bpy

####################################################################################################
#
###      Collision detection script written by Jeet for 5 Minutes Blender YouTube channel        ###
#
####################################################################################################

source_object_name = "greiner_24_wellplate_3300ul"
target_collection_name = "Z-axis"
collision_margin = 0.75
attribute_name = "Collision Frame"
enable_report = True

####################################################################################################

######################## No need to change anything below this line ################################


MAJOR_VERSION, MINOR_VERSION, SUB_VERSION = bpy.app.version

if source_object_name not in bpy.data.objects:
    print(f"Error: Source object '{source_object_name}' not found in scene.")
    raise KeyError(f"Source object '{source_object_name}' not found.")

if target_collection_name not in bpy.data.collections:
    print(f"Error: Target collection '{target_collection_name}' not found in scene.")
    raise KeyError(f"Target collection '{target_collection_name}' not found.")

source_object = bpy.data.objects[source_object_name]
target_coll = bpy.data.collections[target_collection_name]
target_objects = [obj for obj in target_coll.all_objects if obj.type == 'MESH' and obj != source_object]
num_of_objects = len(target_objects)

if num_of_objects == 0:
    print(f"Error: Target collection '{target_collection_name}' contains no eligible mesh objects.")
    raise ValueError(f"Target collection '{target_collection_name}' has no eligible mesh objects.")

print(f"Source object: '{source_object_name}'")
print(f"Target collection: '{target_collection_name}' ({num_of_objects} eligible mesh object(s))")

frame_num = bpy.context.scene.frame_start
frame_curr = bpy.context.scene.frame_current
geo_modifier_list = []
collision_detected = [False] * num_of_objects
collision_count = 0

def create_geometry_node_group():
    geometry_nodes = bpy.data.node_groups.new(type = 'GeometryNodeTree', name = "Geometry Nodes")

    attribute_statistic = geometry_nodes.nodes.new("GeometryNodeAttributeStatistic")
    attribute_statistic.data_type = 'FLOAT'
    attribute_statistic.domain = 'POINT'
    attribute_statistic.inputs[1].default_value = True
    attribute_statistic.inputs[2].default_value = 0.0

    input_socket = geometry_nodes.interface.new_socket('NodeInterfaceInput', in_out='INPUT', socket_type='NodeSocketGeometry')
    input_socket.name = 'Geometry'
    input_socket.attribute_domain = 'POINT'

    input_socket = geometry_nodes.interface.new_socket('NodeInterfaceInput', in_out='INPUT', socket_type='NodeSocketObject')
    input_socket.name = 'Target Object'
    input_socket.attribute_domain = 'POINT'

    group_input = geometry_nodes.nodes.new("NodeGroupInput")
    position = geometry_nodes.nodes.new("GeometryNodeInputPosition")

    geometry_proximity = geometry_nodes.nodes.new("GeometryNodeProximity")
    geometry_proximity.target_element = 'FACES'

    object_info = geometry_nodes.nodes.new("GeometryNodeObjectInfo")
    object_info.transform_space = 'RELATIVE'
    object_info.inputs[1].default_value = False

    store_named_attribute = geometry_nodes.nodes.new("GeometryNodeStoreNamedAttribute")
    store_named_attribute.data_type = 'FLOAT'
    store_named_attribute.domain = 'POINT'
    store_named_attribute.inputs[1].default_value = True
    store_named_attribute.inputs[2].default_value = "Distance"
    store_named_attribute.inputs[3].default_value = 0.0

    output_socket = geometry_nodes.interface.new_socket('NodeInterfaceOutput', in_out='OUTPUT', socket_type='NodeSocketGeometry')
    output_socket.name = 'Geometry'
    output_socket.attribute_domain = 'POINT'

    group_output = geometry_nodes.nodes.new("NodeGroupOutput")

    attribute_statistic.location = (160.26416015625, 216.73521423339844)
    group_input.location = (-629.3379516601562, -13.986187934875488)
    position.location = (-293.7306213378906, -287.6280822753906)
    geometry_proximity.location = (-76.7705307006836, -83.87486267089844)
    object_info.location = (-292.6222229003906, -93.44523620605469)
    store_named_attribute.location = (376.3645935058594, 62.42335891723633)
    group_output.location = (560.0, -12.862828254699707)

    attribute_statistic.width, attribute_statistic.height = 140.0, 100.0
    group_input.width, group_input.height = 140.0, 100.0
    position.width, position.height = 140.0, 100.0
    geometry_proximity.width, geometry_proximity.height = 140.0, 100.0
    object_info.width, object_info.height = 140.0, 100.0
    store_named_attribute.width, store_named_attribute.height = 140.0, 100.0
    group_output.width, group_output.height = 140.0, 100.0

    # Socket indices changed in Blender 4.2
    if (MAJOR_VERSION > 4) or ((MAJOR_VERSION == 4) and (MINOR_VERSION >= 2)):
        geometry_nodes.links.new(object_info.outputs[4], geometry_proximity.inputs[0])
        geometry_nodes.links.new(position.outputs[0], geometry_proximity.inputs[2])
    else:
        geometry_nodes.links.new(object_info.outputs[3], geometry_proximity.inputs[0])
        geometry_nodes.links.new(position.outputs[0], geometry_proximity.inputs[1])

    geometry_nodes.links.new(geometry_proximity.outputs[1], attribute_statistic.inputs[2])
    geometry_nodes.links.new(group_input.outputs[0], attribute_statistic.inputs[0])
    geometry_nodes.links.new(group_input.outputs[1], object_info.inputs[0])
    geometry_nodes.links.new(attribute_statistic.outputs[3], store_named_attribute.inputs[3])
    geometry_nodes.links.new(group_input.outputs[0], store_named_attribute.inputs[0])
    geometry_nodes.links.new(store_named_attribute.outputs[0], group_output.inputs[0])
    return geometry_nodes

geometry_node_group = create_geometry_node_group()

for obj in target_objects:
    modifier = obj.modifiers.new("Geometry Nodes", "NODES")
    modifier.node_group = geometry_node_group
    modifier["Socket_1"] = source_object
    geo_modifier_list.append(modifier)
    if attribute_name in obj:
        del obj[attribute_name]

if enable_report:
    print(f"Processing started under Collision Margin method...")

while (frame_num <= bpy.context.scene.frame_end) and (collision_count < num_of_objects):

    bpy.context.scene.frame_set(frame_num)
    depsgraph = bpy.context.evaluated_depsgraph_get()

    for index, obj in enumerate(target_objects):

        if not collision_detected[index]:
            evaluated_obj = obj.evaluated_get(depsgraph).data
            distance = evaluated_obj.attributes.get('Distance').data[0].value
            # print(f"Frame number {frame_num}")
            # print(f"Distance is {distance}")

            if distance <= collision_margin:
                collision_detected[index] = True
                obj[attribute_name] = frame_num
                collision_count += 1
                if enable_report:
                    print(f"Collision detected for {obj.name}")

    if enable_report:
        print(f"Frame number {frame_num} is processed.")

    frame_num += 1

bpy.context.scene.frame_set(frame_curr)

# Clean-up work.
for obj, modifier in zip(target_objects, geo_modifier_list):
    obj.modifiers.remove(modifier)

bpy.data.node_groups.remove(geometry_node_group, do_unlink=True)

print("Processing completed successfully.")
