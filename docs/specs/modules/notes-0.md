# module deps/inheritance

```py
class Detection2DModule(Module):
    image: In[Image] = None  # type: ignore
    
    detections: Out[Detection2DArray] = None  # type: ignore
    annotations: Out[ImageAnnotations] = None  # type: ignore
```

```py
class Detection3DModule(Detection2DModule):
    detections3d: Out[Detection3DArray] = None  # type: ignore
```

above doesn't work, all I/O needs to be re-specified. 

# configuration inheritance

protocol/service/spec.py - Configurable module built, experimenting with config classes.

seems like dataclasses have inheritance issues (something related to optional values)

```py
class 2DConfiguration():
   key: "some way to specify config typing and also defaults"
 
class Detection2DModule(Module[2DConfiguration]):
   def __init__(self, **kwargs): # <- kwargs are config keys, can this be typed?
      ..._

class 3DConfiguration(2DConfiguration):
   bla: "we inherit 2dConfiguration"

class Detection3DModule(Detection2DModule[how_does_this_work?]):
   def __init__(self, **how_does_typing_work?)_:
   ...
```

# composibility/decomposibility

should be able to run Detection3DModule alone (2d processsing is inherited)
or as 2d + 3d as separate nodes, in some very convinienet way

stream/audio/* stuff is really convininet to use, modules aren't, they should be.

`player(normalizer(microphone_input))` is super fast, building this with streams, then calling dimos.deploy manually would be annoying, modules have full inspectability and they should support this type of an api or something similar.

# Stream generalization

skills are generators, streams between modules are something DIY, streams in modules are reactivex sometimes. 
We need a good general interface for "a stream" that can be interpreted as a generator, rxpy, loop etc. from outside and inside

# performance

all streams should have easy way to test performance speed, we want to detect perforamnce deteriroration in CI, related to testing task below

# testing

All modules should have an easy way to feed "a moment in time" into them, or a full replay, some slice of all streams they care for - to check the module output. - important for testing complex higher order processing without running an actual robot or nodes below. Makes it easy to test performance of the isolated module

