import threading

def start_thread(func):
    def wrapped(*args, **kwargs):
        thread = threading.Thread(target=func, args=args, kwargs=kwargs, daemon=True)
        thread.start()
        return thread
    return wrapped

def standart_position_output(func):
    """"""
    def wrapped(self):
        point = {}
        rotation = {}

        for axis, value in zip(['x', 'y', 'z'], func(self)['position'][:3]):
            point.update({axis: value})
        for rot, value in zip(['roll', 'pitch', 'yaw'], func(self)['position'][3:]):
            rotation.update({rot: value})

        try:
            ref_frame = {'point': point, 'rotation': rotation, 'timestamp': func(self)['timestamp']}
        except KeyError:
            return {'point': point, 'rotation': rotation}
        return ref_frame
    return wrapped