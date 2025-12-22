


- short run file (horrible atm)
a way to think about modules hierarchicaly

- handling manual connections is annoying (for sure needs to be bidirectional)
(you dont need to use .connect, can just specify transport on both ends?)

- type checks on input (don't connect incompatible types)

- dynamic auto-connecting of modules, just init 2 modules that can be hooked up
autoconnect(microphone, normalizer, speaker, transport=LCM) or like transport_f=Callable[[Module,Module],Module]

- can we do microphone().pipe(normalizer()).pipe(speaker) - but with modules somehow? or something close to it?
  
- make sure we can test modules in CI

- good recording and replay (how does this work? should be able to record any module output, and replay from multiple modules in a time synced manner)



connectionmodule.inputs

connectionmodule.outputs

self.dimos = Dimos(
    modules=[
        {
            'module': ConnectionModule,
            'args': (),
            'kwargs': {},
            'transports': {
                'lidar': core.LCMTransport("/lidar", LidarMessage)
                'odom': core.LCMTransport("/odom", PoseStamped)
            },
        },
        {
            'module': Map,
            'args': (),
            'kwargs': {'voxel_size': 0.5},
            'transports': {
                'lidar': core.LCMTransport("/lidar", LidarMessage)
            },
        },
    ],
)

microphone().pipe(normalizer()).pipe(speaker) 


self.dimos.deploy()


new_module = autoconnect(microphone(device=0), normalizer(), speaker(), transport=LCM)

# passive, deployed later
# configs propagade deeply
# how do we specify transports deeply for modules, individual streams etc
smart_go2 = autoconnect(go2_connection, go2_agent(transport=lcm), depth_modile, navigation)

# we need a way to define transports on the stream level,
# module level, and robot level

smart_go2(
   frame_id_prefix: "" # <- think about this
   go2_agent: { transport: lcm }
)

# for paul potentially
smart_go2.render_yaml()
autoconnect(load_yaml())

deploy(smart_go2) # ?

# mega_smart_go2 is a module on the outside, has inputs, outputs, rpcs etc
mega_smart_go2 = autoconnect(smart_go2(), microphone())

# think how would a user conviniently override specifics? multiple audio outputs, which one is used for tts?



