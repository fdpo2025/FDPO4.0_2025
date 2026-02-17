import yaml

#-------------------#
# load file
#-------------------#
def load_file(yaml_file):
    with open(yaml_file, "r") as f:
        file = yaml.safe_load(f)
    return file

# class to wrap dict or list
class InlineDictList:
    def __init__(self, value):
        self.value = value

# class to dump specific data structures inline
class InlineDumper(yaml.SafeDumper):
    pass

# function to represent InlineDictList objects inline
def _f_represent_dict_list(dumper, data):
    if isinstance(data.value, dict):
        return dumper.represent_mapping('tag:yaml.org,2002:map', 
                                        data.value, 
                                        flow_style=True)
    elif isinstance(data.value, list):
        return dumper.represent_sequence('tag:yaml.org,2002:seq', 
                                         data.value,
                                         flow_style=True)
    raise TypeError(f"InlineDictList only supports dict or list, got {type(data.value)}")

# register representer (call once)
_INLINE_REGISTERED = False
def register_InlineDumper():
    global _INLINE_REGISTERED
    if not _INLINE_REGISTERED:
        InlineDumper.add_representer(InlineDictList, _f_represent_dict_list)
        _INLINE_REGISTERED = True

# function to wrap dict or list to be written inline
def get_inline_dict_list(data):
    return InlineDictList(data)

#------------------#
# save file
#------------------#
def save_file(yaml_file, data):
    
    # register the InlineDumper class
    register_InlineDumper()

    with open(yaml_file, "w", encoding="utf-8") as f:
        yaml.dump(
            data,
            f,
            Dumper=InlineDumper,
            sort_keys=False,
            default_flow_style=False,  # with new lines for default structures
            allow_unicode=True,
        )

"""
usage:

data = {
    "a": get_inline_dict_list({"x": 1, "y": 2}),
    "b": get_inline_dict_list([1, 2, 3]),
}
save_file("file.yaml", data)
"""