import bpy
import numpy as np
from mathutils import Vector
import bmesh
from dataclasses import dataclass,field
from enum import Enum, auto
import blender_plots as bplt
import csv

gantry = bpy.data.collections["gantry"]
xy_carriage = bpy.data.collections["XY-carriage"]
y_parts = bpy.data.collections["Y-parts"]

# Path to the CSV file (relative to the .blend file)
csv_path = bpy.path.abspath("//from_gcode/pathout.csv")
tool_data_path = bpy.path.abspath("//from_gcode/tool_data.csv")

class GantryHead(int,Enum):
    TOOL0 = 0
    TOOL1 = 1
    TOOL2 = 2
    TOOL3 = 3
    NOTOOL = -1

@dataclass
class CollisionEvent:
    frame: int
    collided_object: object
    location: Vector

@dataclass
class AnimationData:
    tool_agenda:  list[int]    = field(default_factory=list)
    hull_origins: np.ndarray   = field(default_factory=lambda: np.array([]))
    directions:   np.ndarray   = field(default_factory=lambda: np.array([]))
    frame_num:    int          = 0
    
    

_tool_hulls_dict={
    GantryHead.NOTOOL : None,
    GantryHead.TOOL0 : None,
    GantryHead.TOOL1 : None,
    GantryHead.TOOL2 : None,
    GantryHead.TOOL3 : None,
}

_target_dict={ 
    GantryHead.NOTOOL : [gantry,xy_carriage],
    GantryHead.TOOL0 : [],
    GantryHead.TOOL1 : [],
    GantryHead.TOOL2 : [],
    GantryHead.TOOL3 : [],
}

_vision_dict={
    GantryHead.NOTOOL : [gantry,xy_carriage,y_parts],
    GantryHead.TOOL0 : [],
    GantryHead.TOOL1 : [],
    GantryHead.TOOL2 : [],
    GantryHead.TOOL3 : [],
}

_collision_list : list[CollisionEvent] = []

_tool_changing_dict: dict[int,(int,str)] = {1: (-1, "Gantry")}

def frame_update(scene):
    global _tool_hulls_dict

    with open(csv_path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        tools = []
        for row in reader:
            tools.append(row[5])
    
    for frame in range(scene.frame_start,scene.frame_end):
        frame_name = f"frame_{frame}_directions"
        obj = bpy.data.objects.get(frame_name)
        if obj is not None:
            obj.hide_set(frame != scene.frame_current)
            obj.hide_render = (frame != scene.frame_current)
            
    
    for hull_idx,hull in _tool_hulls_dict.items():
        if hull is not None:
            hull.hide_set(int(float(tools[scene.frame_current-1])) != hull_idx )
            hull.hide_render = (int(float(tools[scene.frame_current-1])) != hull_idx)




def get_or_create_collection(name):
    if name in bpy.data.collections:
        return bpy.data.collections[name]
    col = bpy.data.collections.new(name)
    bpy.context.scene.collection.children.link(col)
    return col

def remove_collection(name):
    col = bpy.data.collections.get(name)
    if col is None:
        return
    
    for parent in bpy.data.collections:
        if col.name in parent.children:
            parent.children.unlink(col)
    if col.name in bpy.context.scene.collection.children:
        bpy.context.scene.collection.children.unlink(col)

    for obj in col.objects[:]:
        bpy.data.objects.remove(obj, do_unlink=True)
    bpy.data.collections.remove(col)

def _extract_tool_changing(animation:AnimationData):
    global _tool_changing_dict

    frame = 0
    tool_id = 0
    tool_name = ""
    tool_agenda = []

    with open(csv_path, newline='') as csvfile:
        reader_0 = csv.reader(csvfile)
        for row_i in reader_0:
            tool_agenda.append(int(float(row_i[5])))
            if row_i[4] == "1.0": #tool locking identified

                print("-"*60)
                print("INTERPRETED ROWS")
                print("-"*60)
                print(f"PATH:{row_i}")

                tool_id = int(float(row_i[5]))
                
                frame = reader_0.line_num

                with open(tool_data_path, newline='') as datafile:

                    reader_1 = csv.reader(datafile)
                    next(reader_1)

                    for row_j in reader_1:
                        
                        if  row_j[0] == str(tool_id):

                            print(f"TOOL:{row_j}")
                            tool_name = row_j[1] 
        
        _tool_changing_dict[frame] = (tool_id,tool_name)
        animation.tool_agenda = tool_agenda
    print(f"FINAL DICT:{_tool_changing_dict}")
    print("-"*60)


def _gen_obj_hull(obj_list,anchor, name, depsgraph):

    anchor_matrix_inv = anchor.evaluated_get(depsgraph).matrix_world.inverted()
    combined_bm=bmesh.new()

    for obj in obj_list:
        obj_eval = obj.evaluated_get(depsgraph)
        temp_bm = bmesh.new()
        temp_bm.from_object(obj_eval, depsgraph)
        transform = anchor_matrix_inv @ obj_eval.matrix_world
        for v in temp_bm.verts:
            combined_bm.verts.new(transform @ v.co) 
        obj_eval.to_mesh_clear()
        temp_bm.free()

    
    combined_bm.verts.ensure_lookup_table()
    hull_result = bmesh.ops.convex_hull(combined_bm, input=combined_bm.verts[:])

    interior_geom = hull_result.get("geom_interior", [])
    unused_geom   = hull_result.get("geom_unused", [])
    delete_geom   = list(set(interior_geom + unused_geom))

    bmesh.ops.delete(combined_bm, geom=delete_geom, context='VERTS')
    
    hull_mesh=bpy.data.meshes.new(name)
    combined_bm.to_mesh(hull_mesh)
    combined_bm.free()

    obj_hull = bpy.data.objects.new(name, hull_mesh)
    bpy.context.scene.collection.objects.link(obj_hull) 

    return obj_hull,hull_mesh

def _gen_convex_hull(target_list,name,depsgraph):

    target_parts = []

    for target in target_list:
        if not isinstance(target,bpy.types.Collection):
            raise(ValueError)
        else:
            target_parts.extend([
                obj for obj in target.all_objects if obj.type == 'MESH'])

    anchor = target_parts[0]
    obj_hull, hull_mesh = _gen_obj_hull(target_parts,
                                        anchor=anchor,
                                        depsgraph=depsgraph,
                                        name=name
                                        )
    

    obj_hull.constraints.new('COPY_TRANSFORMS')
    obj_hull.constraints['Copy Transforms'].target = target_parts[0]


    target_list[0].objects.link(obj_hull)
    bpy.ops.object.origin_set(type = 'ORIGIN_GEOMETRY')

    return  obj_hull, hull_mesh

def _gen_tool_hulls(scene,depsgraph):
    global _tool_changing_dict
    global _target_dict
    global _vision_dict

    remove_collection("Tool Hulls")

    print("-"*60)
    print("TOOL HULLS DEBUG")
    print("-"*60)
    print(f"DICT:{_tool_changing_dict}")

    col = get_or_create_collection("Tool Hulls")
    for frame in _tool_changing_dict.keys():

        scene.frame_set(frame)

        current_tool_id, current_tool_name = _tool_changing_dict[frame]
        current_tool_enum = GantryHead(current_tool_id)

        print(current_tool_enum)

        coll_names=[coll.name for coll in bpy.data.collections]

        if current_tool_name in coll_names:
            current_tool =  bpy.data.collections[current_tool_name]
            _target_dict[current_tool_enum] = _target_dict[GantryHead.NOTOOL] + [current_tool]
            _vision_dict[current_tool_enum] = _vision_dict[GantryHead.NOTOOL] + [current_tool]        

        print("-"*60)
        print(f"TARGET DICT:{_target_dict[current_tool_id]}")
        print(f"VISION DICT:{_vision_dict[current_tool_id]}")
        print("-"*60)

        if current_tool_name+"_hull" not in bpy.data.collections["Tool Hulls"].all_objects:
            obj_hull,hull_mesh = _gen_convex_hull(_target_dict[current_tool_id],current_tool_name+"_hull",depsgraph)
            
            _tool_hulls_dict[current_tool_enum] = obj_hull
            bpy.data.collections["Tool Hulls"].objects.link(obj_hull)
            
        for obj in bpy.data.collections["Tool Hulls"].all_objects:
            obj.hide_set(True)            

    print(f"HULL DICT:{_tool_hulls_dict}")

def _hide_vision_collection(vision_list):

    for collection in vision_list:
        for obj in collection.all_objects:
            obj.hide_set(True)

def _unhide_vision_collection(vision_list):

    for collection in vision_list:
        for obj in collection.all_objects:
            obj.hide_set(False)

def _compute_directions_CH(scene,frame_num):

    with open(csv_path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        tools = []
        for row in reader:
            tools.append(row[5])

    directions = np.zeros((frame_num,3))
    hull_origins = np.zeros((frame_num,3))

    for frame,i in zip(range(1,frame_num+1),range(frame_num)):
        scene.frame_set(frame)
        bpy.context.view_layer.update()
        depsgraph = bpy.context.evaluated_depsgraph_get()

        obj_hull = _tool_hulls_dict[int(float(tools[frame-1]))]
        obj_eval = obj_hull.evaluated_get(depsgraph)

        hull_origins[i] = obj_eval.matrix_world.translation.copy()

    for frame, i in zip(range(1,frame_num+1), range(frame_num)):
        scene.frame_set(frame)
        bpy.context.view_layer.update()
        depsgraph = bpy.context.evaluated_depsgraph_get()
        
        if frame < (frame_num-1):
            directions[frame] = hull_origins[i+1]-hull_origins[i]

    return hull_origins,directions

def _trace_ray_traj(frame_num,origins,directions,frame):

    coll_name = "Ray Tracing Directions"
    coll = bpy.data.collections.get(coll_name)
    if coll is None:
        coll = bpy.data.collections.new(coll_name)
        bpy.context.scene.collection.children.link(coll)

    colors = [c for c in np.linspace((0, 0, 1), (1, 0, 0), num=frame_num, dtype=float)]

    arrows = bplt.Arrows(origins,
                np.tile(directions[frame-1], (len(origins), 1)),
                color=colors[frame-1],
                name=f"frame_{frame}_directions",
                head_length=0.001,
                radius=.0005,
                radius_ratio=2,
                end_trim_length=0)
    
    if arrows is not None:
        blender_obj = bpy.data.objects.get(f"frame_{frame}_directions")
        if blender_obj is not None:
            if blender_obj.name in bpy.context.scene.collection.objects:
                bpy.context.scene.collection.objects.unlink(blender_obj)
            coll.objects.link(blender_obj)
            blender_obj.hide_set(True)
        
            
    
def _ray_tracing(scene,depsgraph,frame,vertices,directions):

    distance = np.linalg.norm(directions)
    if distance < 1e-6:
        return 
    direction_normalized = Vector(directions / distance)

    obj_collided = []

    for coord in vertices:
            hit, loc, normal, index, object, mat = scene.ray_cast(
                depsgraph = depsgraph, 
                origin = Vector(coord),
                direction = direction_normalized,
                distance = distance
                )
            if hit and object.name not in obj_collided:
                obj_collided.append(object.name)
                print(f"Collision between gantry and {object.name} during frame {frame}")

def ray_tracing_CD():

    global _target_dict,_vision_dict,_tool_changing_dict,_hull_verts_dict
    anim = AnimationData()

    bpy.context.view_layer.update()
    scene = bpy.context.scene
    depsgraph = bpy.context.evaluated_depsgraph_get()

    anim.frame_num = scene.frame_end - scene.frame_start + 1   

    _extract_tool_changing(anim)

    _gen_tool_hulls(scene,depsgraph)     

    anim.hull_origins, anim.directions = _compute_directions_CH(scene,anim.frame_num)

    for frame in range(scene.frame_start,scene.frame_end):
        scene.frame_set(frame)
        bpy.context.evaluated_depsgraph_get()


        print("-"*60)
        print(f"FRAME {frame}")
        print("-"*60)

        hull = _tool_hulls_dict[anim.tool_agenda[frame-1]]
        print(f" CURRENT HULL FOR RT: {hull.name}")

        vertices = np.array([hull.matrix_world @ v.co for v in hull.data.vertices])
        print(f"VERTICES OF HULL:{vertices.shape}")

        _hide_vision_collection(_vision_dict[anim.tool_agenda[frame-1]])
        _ray_tracing(scene,depsgraph,frame,vertices,anim.directions[frame])
        _unhide_vision_collection(_vision_dict[anim.tool_agenda[frame-1]])

        _trace_ray_traj(anim.frame_num,vertices,anim.directions,frame)

        bpy.app.handlers.frame_change_pre.append(frame_update)






    




        
        