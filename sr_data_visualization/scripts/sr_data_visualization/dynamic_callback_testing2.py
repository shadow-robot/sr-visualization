
def make_control_loop_callback(value_to_print):
    def _function():
        print value_to_print
    return _function

#You can generate a list of these and store, again at runtime.

control_loop_callbacks = [make_control_loop_callback(i) for i in range(1, 11)]
for each in control_loop_callbacks:
    each()
bsreak = 1


# #The solution here is to create a namespace by, e.g., using collections.namedtuple:
#
#     variables = namedtuple('Variables', names)._make(0 for _ in names)
#
# #Now, you can access them statically:
#
#     if cmd == 'h':
#         return (variables.a**2 + variables.b**2) ** .5