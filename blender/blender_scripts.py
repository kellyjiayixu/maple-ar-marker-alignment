
def blenderCamToOpenCVParams(camName="Camera"):
    # get the relevant data
    cam = bpy.data.objects[camName].data
    scene = bpy.context.scene
    # assume image is not scaled
    assert scene.render.resolution_percentage == 100
    # assume angles describe the horizontal field of view
    assert cam.sensor_fit != 'VERTICAL'
    #
    f_in_mm = cam.lens
    sensor_width_in_mm = cam.sensor_width
    #
    w = scene.render.resolution_x
    h = scene.render.resolution_y
    #
    pixel_aspect = scene.render.pixel_aspect_y / scene.render.pixel_aspect_x
    #
    f_x = f_in_mm / sensor_width_in_mm * w
    f_y = f_x * pixel_aspect
    # yes, shift_x is inverted. WTF blender?
    c_x = w * (0.5 - cam.shift_x)
    # and shift_y is still a percentage of width..
    c_y = h * 0.5 + w * cam.shift_y
    # Output
    K = [[f_x, 0, c_x],
        [0, f_y, c_y],
        [0,   0,   1]]
    return K