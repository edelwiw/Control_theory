from manim import * 
import math 

config.frame_height = 8 * 2
config.frame_width = 14.22 * 2

D = 0.65

class ContinuousMotion(MovingCameraScene):
    def construct(self):

        self.camera.frame.set(width=30)


        def func(pos):
            theta = pos[0]; v = pos[1]
            res = np.array([(v ** 2 - math.cos(theta)) / v , -math.sin(theta) - D * v ** 2, 1.0])
            return res

        stream_lines = StreamLines(func, stroke_width=2, max_anchors_per_line=40, virtual_time=3, min_color_scheme_value=2, max_color_scheme_value=30)

        ax = Axes(x_range=[-14, 14, 1], y_range=[-7, 7, 1],  x_length = 28, y_length=14, axis_config={"include_numbers": True})
        labels = ax.get_axis_labels(MathTex(r"\Theta"), MathTex(r"\vartheta"))

        text = MathTex(r"D = ", str(D)).to_corner(UL).scale(1.5)

        self.add(stream_lines, ax, labels, text)
        stream_lines.start_animation(warm_up=False, flow_speed=1)
        self.wait(stream_lines.virtual_time / stream_lines.flow_speed)