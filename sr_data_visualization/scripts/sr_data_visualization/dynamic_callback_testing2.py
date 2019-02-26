
def make_control_loop_callback(value_to_print):
    def _function():
        print value_to_print
    return _function

#You can generate a list of these and store, again at runtime.

control_loop_callbacks = [make_control_loop_callback(i) for i in range(1, 11)]
for each in control_loop_callbacks:
    each()
bsreak = 1