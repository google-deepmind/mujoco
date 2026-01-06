.. dropdown:: :ref:`mujoco<mujoco>`
   :icon: star
   :open:

   .. list-table::
      :class: mjcf-attributes
      :widths: 25 25 25 25

      * - :ref:`model<mujoco-model>`
        -
        -
        -

   .. dropdown:: :ref:`option<option>`

      .. list-table::
         :class: mjcf-attributes
         :widths: 25 25 25 25

         * - :ref:`timestep<option-timestep>`
           - :ref:`impratio<option-impratio>`
           - :ref:`tolerance<option-tolerance>`
           - :ref:`ls_tolerance<option-ls_tolerance>`
         * - :ref:`noslip_tolerance<option-noslip_tolerance>`
           - :ref:`ccd_tolerance<option-ccd_tolerance>`
           - :ref:`sleep_tolerance<option-sleep_tolerance>`
           - :ref:`gravity<option-gravity>`
         * - :ref:`wind<option-wind>`
           - :ref:`magnetic<option-magnetic>`
           - :ref:`density<option-density>`
           - :ref:`viscosity<option-viscosity>`
         * - :ref:`o_margin<option-o_margin>`
           - :ref:`o_solref<option-o_solref>`
           - :ref:`o_solimp<option-o_solimp>`
           - :ref:`o_friction<option-o_friction>`
         * - :ref:`integrator<option-integrator>`
           - :ref:`cone<option-cone>`
           - :ref:`jacobian<option-jacobian>`
           - :ref:`solver<option-solver>`
         * - :ref:`iterations<option-iterations>`
           - :ref:`ls_iterations<option-ls_iterations>`
           - :ref:`noslip_iterations<option-noslip_iterations>`
           - :ref:`ccd_iterations<option-ccd_iterations>`
         * - :ref:`sdf_iterations<option-sdf_iterations>`
           - :ref:`sdf_initpoints<option-sdf_initpoints>`
           - :ref:`actuatorgroupdisable<option-actuatorgroupdisable>`
           -

      .. dropdown:: :ref:`flag<option-flag>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`constraint<option-flag-constraint>`
              - :ref:`equality<option-flag-equality>`
              - :ref:`frictionloss<option-flag-frictionloss>`
              - :ref:`limit<option-flag-limit>`
            * - :ref:`contact<option-flag-contact>`
              - :ref:`spring<option-flag-spring>`
              - :ref:`damper<option-flag-damper>`
              - :ref:`gravity<option-flag-gravity>`
            * - :ref:`clampctrl<option-flag-clampctrl>`
              - :ref:`warmstart<option-flag-warmstart>`
              - :ref:`filterparent<option-flag-filterparent>`
              - :ref:`actuation<option-flag-actuation>`
            * - :ref:`refsafe<option-flag-refsafe>`
              - :ref:`sensor<option-flag-sensor>`
              - :ref:`midphase<option-flag-midphase>`
              - :ref:`eulerdamp<option-flag-eulerdamp>`
            * - :ref:`autoreset<option-flag-autoreset>`
              - :ref:`nativeccd<option-flag-nativeccd>`
              - :ref:`island<option-flag-island>`
              - :ref:`override<option-flag-override>`
            * - :ref:`energy<option-flag-energy>`
              - :ref:`fwdinv<option-flag-fwdinv>`
              - :ref:`invdiscrete<option-flag-invdiscrete>`
              - :ref:`multiccd<option-flag-multiccd>`
            * - :ref:`sleep<option-flag-sleep>`
              -
              -
              -

   .. dropdown:: :ref:`compiler<compiler>`

      .. list-table::
         :class: mjcf-attributes
         :widths: 25 25 25 25

         * - :ref:`autolimits<compiler-autolimits>`
           - :ref:`boundmass<compiler-boundmass>`
           - :ref:`boundinertia<compiler-boundinertia>`
           - :ref:`settotalmass<compiler-settotalmass>`
         * - :ref:`balanceinertia<compiler-balanceinertia>`
           - :ref:`strippath<compiler-strippath>`
           - :ref:`coordinate<compiler-coordinate>`
           - :ref:`angle<compiler-angle>`
         * - :ref:`fitaabb<compiler-fitaabb>`
           - :ref:`eulerseq<compiler-eulerseq>`
           - :ref:`meshdir<compiler-meshdir>`
           - :ref:`texturedir<compiler-texturedir>`
         * - :ref:`discardvisual<compiler-discardvisual>`
           - :ref:`usethread<compiler-usethread>`
           - :ref:`fusestatic<compiler-fusestatic>`
           - :ref:`inertiafromgeom<compiler-inertiafromgeom>`
         * - :ref:`inertiagrouprange<compiler-inertiagrouprange>`
           - :ref:`saveinertial<compiler-saveinertial>`
           - :ref:`assetdir<compiler-assetdir>`
           - :ref:`alignfree<compiler-alignfree>`

      .. dropdown:: :ref:`lengthrange<compiler-lengthrange>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`mode<compiler-lengthrange-mode>`
              - :ref:`useexisting<compiler-lengthrange-useexisting>`
              - :ref:`uselimit<compiler-lengthrange-uselimit>`
              - :ref:`accel<compiler-lengthrange-accel>`
            * - :ref:`maxforce<compiler-lengthrange-maxforce>`
              - :ref:`timeconst<compiler-lengthrange-timeconst>`
              - :ref:`timestep<compiler-lengthrange-timestep>`
              - :ref:`inttotal<compiler-lengthrange-inttotal>`
            * - :ref:`interval<compiler-lengthrange-interval>`
              - :ref:`tolrange<compiler-lengthrange-tolrange>`
              -
              -

   .. dropdown:: :ref:`size<size>`

      .. list-table::
         :class: mjcf-attributes
         :widths: 25 25 25 25

         * - :ref:`memory<size-memory>`
           - :ref:`njmax<size-njmax>`
           - :ref:`nconmax<size-nconmax>`
           - :ref:`nstack<size-nstack>`
         * - :ref:`nuserdata<size-nuserdata>`
           - :ref:`nkey<size-nkey>`
           - :ref:`nuser_body<size-nuser_body>`
           - :ref:`nuser_jnt<size-nuser_jnt>`
         * - :ref:`nuser_geom<size-nuser_geom>`
           - :ref:`nuser_site<size-nuser_site>`
           - :ref:`nuser_cam<size-nuser_cam>`
           - :ref:`nuser_tendon<size-nuser_tendon>`
         * - :ref:`nuser_actuator<size-nuser_actuator>`
           - :ref:`nuser_sensor<size-nuser_sensor>`
           -
           -

   .. dropdown:: :ref:`statistic<statistic>`

      .. list-table::
         :class: mjcf-attributes
         :widths: 25 25 25 25

         * - :ref:`meaninertia<statistic-meaninertia>`
           - :ref:`meanmass<statistic-meanmass>`
           - :ref:`meansize<statistic-meansize>`
           - :ref:`extent<statistic-extent>`
         * - :ref:`center<statistic-center>`
           -
           -
           -

   .. dropdown:: :ref:`asset<asset>`


      .. dropdown:: :ref:`mesh<asset-mesh>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<asset-mesh-name>`
              - :ref:`class<asset-mesh-class>`
              - :ref:`content_type<asset-mesh-content_type>`
              - :ref:`file<asset-mesh-file>`
            * - :ref:`vertex<asset-mesh-vertex>`
              - :ref:`normal<asset-mesh-normal>`
              - :ref:`texcoord<asset-mesh-texcoord>`
              - :ref:`face<asset-mesh-face>`
            * - :ref:`refpos<asset-mesh-refpos>`
              - :ref:`refquat<asset-mesh-refquat>`
              - :ref:`scale<asset-mesh-scale>`
              - :ref:`smoothnormal<asset-mesh-smoothnormal>`
            * - :ref:`maxhullvert<asset-mesh-maxhullvert>`
              - :ref:`inertia<asset-mesh-inertia>`
              - :ref:`builtin<asset-mesh-builtin>`
              - :ref:`params<asset-mesh-params>`
            * - :ref:`material<asset-mesh-material>`
              -
              -
              -

         .. dropdown:: :ref:`plugin<mesh-plugin>`

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`plugin<mesh-plugin-plugin>`
                 - :ref:`instance<mesh-plugin-instance>`
                 -
                 -

            .. dropdown:: :ref:`config<plugin-config>`

               .. list-table::
                  :class: mjcf-attributes
                  :widths: 25 25 25 25

                  * - :ref:`key<plugin-config-key>`
                    - :ref:`value<plugin-config-value>`
                    -
                    -

      .. dropdown:: :ref:`hfield<asset-hfield>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<asset-hfield-name>`
              - :ref:`content_type<asset-hfield-content_type>`
              - :ref:`file<asset-hfield-file>`
              - :ref:`nrow<asset-hfield-nrow>`
            * - :ref:`ncol<asset-hfield-ncol>`
              - :ref:`size<asset-hfield-size>`
              - :ref:`elevation<asset-hfield-elevation>`
              -

      .. dropdown:: :ref:`skin<asset-skin>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<asset-skin-name>`
              - :ref:`file<asset-skin-file>`
              - :ref:`material<asset-skin-material>`
              - :ref:`rgba<asset-skin-rgba>`
            * - :ref:`inflate<asset-skin-inflate>`
              - :ref:`vertex<asset-skin-vertex>`
              - :ref:`texcoord<asset-skin-texcoord>`
              - :ref:`face<asset-skin-face>`
            * - :ref:`group<asset-skin-group>`
              -
              -
              -

         .. dropdown:: :ref:`bone<skin-bone>`

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`body<skin-bone-body>`
                 - :ref:`bindpos<skin-bone-bindpos>`
                 - :ref:`bindquat<skin-bone-bindquat>`
                 - :ref:`vertid<skin-bone-vertid>`
               * - :ref:`vertweight<skin-bone-vertweight>`
                 -
                 -
                 -

      .. dropdown:: :ref:`texture<asset-texture>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<asset-texture-name>`
              - :ref:`type<asset-texture-type>`
              - :ref:`colorspace<asset-texture-colorspace>`
              - :ref:`content_type<asset-texture-content_type>`
            * - :ref:`file<asset-texture-file>`
              - :ref:`gridsize<asset-texture-gridsize>`
              - :ref:`gridlayout<asset-texture-gridlayout>`
              - :ref:`fileright<asset-texture-fileright>`
            * - :ref:`fileleft<asset-texture-fileleft>`
              - :ref:`fileup<asset-texture-fileup>`
              - :ref:`filedown<asset-texture-filedown>`
              - :ref:`filefront<asset-texture-filefront>`
            * - :ref:`fileback<asset-texture-fileback>`
              - :ref:`builtin<asset-texture-builtin>`
              - :ref:`rgb1<asset-texture-rgb1>`
              - :ref:`rgb2<asset-texture-rgb2>`
            * - :ref:`mark<asset-texture-mark>`
              - :ref:`markrgb<asset-texture-markrgb>`
              - :ref:`random<asset-texture-random>`
              - :ref:`width<asset-texture-width>`
            * - :ref:`height<asset-texture-height>`
              - :ref:`hflip<asset-texture-hflip>`
              - :ref:`vflip<asset-texture-vflip>`
              - :ref:`nchannel<asset-texture-nchannel>`

      .. dropdown:: :ref:`material<asset-material>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<asset-material-name>`
              - :ref:`class<asset-material-class>`
              - :ref:`texture<asset-material-texture>`
              - :ref:`texrepeat<asset-material-texrepeat>`
            * - :ref:`texuniform<asset-material-texuniform>`
              - :ref:`emission<asset-material-emission>`
              - :ref:`specular<asset-material-specular>`
              - :ref:`shininess<asset-material-shininess>`
            * - :ref:`reflectance<asset-material-reflectance>`
              - :ref:`metallic<asset-material-metallic>`
              - :ref:`roughness<asset-material-roughness>`
              - :ref:`rgba<asset-material-rgba>`

         .. dropdown:: :ref:`layer<material-layer>`

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`texture<material-layer-texture>`
                 - :ref:`role<material-layer-role>`
                 -
                 -

      .. dropdown:: :ref:`model<asset-model>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<asset-model-name>`
              - :ref:`file<asset-model-file>`
              - :ref:`content_type<asset-model-content_type>`
              -

   .. dropdown:: :ref:`(world)body<body>`
      :icon: sync

      .. list-table::
         :class: mjcf-attributes
         :widths: 25 25 25 25

         * - :ref:`name<body-name>`
           - :ref:`childclass<body-childclass>`
           - :ref:`pos<body-pos>`
           - :ref:`quat<body-quat>`
         * - :ref:`mocap<body-mocap>`
           - :ref:`axisangle<body-axisangle>`
           - :ref:`xyaxes<body-xyaxes>`
           - :ref:`zaxis<body-zaxis>`
         * - :ref:`euler<body-euler>`
           - :ref:`gravcomp<body-gravcomp>`
           - :ref:`sleep<body-sleep>`
           - :ref:`user<body-user>`

      .. dropdown:: :ref:`inertial<body-inertial>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`pos<body-inertial-pos>`
              - :ref:`quat<body-inertial-quat>`
              - :ref:`mass<body-inertial-mass>`
              - :ref:`diaginertia<body-inertial-diaginertia>`
            * - :ref:`axisangle<body-inertial-axisangle>`
              - :ref:`xyaxes<body-inertial-xyaxes>`
              - :ref:`zaxis<body-inertial-zaxis>`
              - :ref:`euler<body-inertial-euler>`
            * - :ref:`fullinertia<body-inertial-fullinertia>`
              -
              -
              -

      .. dropdown:: :ref:`joint<body-joint>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<body-joint-name>`
              - :ref:`class<body-joint-class>`
              - :ref:`type<body-joint-type>`
              - :ref:`group<body-joint-group>`
            * - :ref:`pos<body-joint-pos>`
              - :ref:`axis<body-joint-axis>`
              - :ref:`springdamper<body-joint-springdamper>`
              - :ref:`limited<body-joint-limited>`
            * - :ref:`actuatorfrclimited<body-joint-actuatorfrclimited>`
              - :ref:`solreflimit<body-joint-solreflimit>`
              - :ref:`solimplimit<body-joint-solimplimit>`
              - :ref:`solreffriction<body-joint-solreffriction>`
            * - :ref:`solimpfriction<body-joint-solimpfriction>`
              - :ref:`stiffness<body-joint-stiffness>`
              - :ref:`range<body-joint-range>`
              - :ref:`actuatorfrcrange<body-joint-actuatorfrcrange>`
            * - :ref:`actuatorgravcomp<body-joint-actuatorgravcomp>`
              - :ref:`margin<body-joint-margin>`
              - :ref:`ref<body-joint-ref>`
              - :ref:`springref<body-joint-springref>`
            * - :ref:`armature<body-joint-armature>`
              - :ref:`damping<body-joint-damping>`
              - :ref:`frictionloss<body-joint-frictionloss>`
              - :ref:`user<body-joint-user>`

      .. dropdown:: :ref:`freejoint<body-freejoint>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<body-freejoint-name>`
              - :ref:`group<body-freejoint-group>`
              - :ref:`align<body-freejoint-align>`
              -

      .. dropdown:: :ref:`geom<body-geom>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<body-geom-name>`
              - :ref:`class<body-geom-class>`
              - :ref:`type<body-geom-type>`
              - :ref:`contype<body-geom-contype>`
            * - :ref:`conaffinity<body-geom-conaffinity>`
              - :ref:`condim<body-geom-condim>`
              - :ref:`group<body-geom-group>`
              - :ref:`priority<body-geom-priority>`
            * - :ref:`size<body-geom-size>`
              - :ref:`material<body-geom-material>`
              - :ref:`friction<body-geom-friction>`
              - :ref:`mass<body-geom-mass>`
            * - :ref:`density<body-geom-density>`
              - :ref:`shellinertia<body-geom-shellinertia>`
              - :ref:`solmix<body-geom-solmix>`
              - :ref:`solref<body-geom-solref>`
            * - :ref:`solimp<body-geom-solimp>`
              - :ref:`margin<body-geom-margin>`
              - :ref:`gap<body-geom-gap>`
              - :ref:`fromto<body-geom-fromto>`
            * - :ref:`pos<body-geom-pos>`
              - :ref:`quat<body-geom-quat>`
              - :ref:`axisangle<body-geom-axisangle>`
              - :ref:`xyaxes<body-geom-xyaxes>`
            * - :ref:`zaxis<body-geom-zaxis>`
              - :ref:`euler<body-geom-euler>`
              - :ref:`hfield<body-geom-hfield>`
              - :ref:`mesh<body-geom-mesh>`
            * - :ref:`fitscale<body-geom-fitscale>`
              - :ref:`rgba<body-geom-rgba>`
              - :ref:`fluidshape<body-geom-fluidshape>`
              - :ref:`fluidcoef<body-geom-fluidcoef>`
            * - :ref:`user<body-geom-user>`
              -
              -
              -

         .. dropdown:: :ref:`plugin<geom-plugin>`

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`plugin<geom-plugin-plugin>`
                 - :ref:`instance<geom-plugin-instance>`
                 -
                 -

            .. dropdown:: :ref:`config<plugin-config>`

               .. list-table::
                  :class: mjcf-attributes
                  :widths: 25 25 25 25

                  * - :ref:`key<plugin-config-key>`
                    - :ref:`value<plugin-config-value>`
                    -
                    -

      .. dropdown:: :ref:`attach<body-attach>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`model<body-attach-model>`
              - :ref:`body<body-attach-body>`
              - :ref:`prefix<body-attach-prefix>`
              -

      .. dropdown:: :ref:`site<body-site>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<body-site-name>`
              - :ref:`class<body-site-class>`
              - :ref:`type<body-site-type>`
              - :ref:`group<body-site-group>`
            * - :ref:`pos<body-site-pos>`
              - :ref:`quat<body-site-quat>`
              - :ref:`material<body-site-material>`
              - :ref:`size<body-site-size>`
            * - :ref:`fromto<body-site-fromto>`
              - :ref:`axisangle<body-site-axisangle>`
              - :ref:`xyaxes<body-site-xyaxes>`
              - :ref:`zaxis<body-site-zaxis>`
            * - :ref:`euler<body-site-euler>`
              - :ref:`rgba<body-site-rgba>`
              - :ref:`user<body-site-user>`
              -

      .. dropdown:: :ref:`camera<body-camera>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<body-camera-name>`
              - :ref:`class<body-camera-class>`
              - :ref:`projection<body-camera-projection>`
              - :ref:`fovy<body-camera-fovy>`
            * - :ref:`ipd<body-camera-ipd>`
              - :ref:`resolution<body-camera-resolution>`
              - :ref:`pos<body-camera-pos>`
              - :ref:`quat<body-camera-quat>`
            * - :ref:`axisangle<body-camera-axisangle>`
              - :ref:`xyaxes<body-camera-xyaxes>`
              - :ref:`zaxis<body-camera-zaxis>`
              - :ref:`euler<body-camera-euler>`
            * - :ref:`mode<body-camera-mode>`
              - :ref:`target<body-camera-target>`
              - :ref:`focal<body-camera-focal>`
              - :ref:`focalpixel<body-camera-focalpixel>`
            * - :ref:`principal<body-camera-principal>`
              - :ref:`principalpixel<body-camera-principalpixel>`
              - :ref:`sensorsize<body-camera-sensorsize>`
              - :ref:`user<body-camera-user>`

      .. dropdown:: :ref:`light<body-light>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<body-light-name>`
              - :ref:`class<body-light-class>`
              - :ref:`directional<body-light-directional>`
              - :ref:`type<body-light-type>`
            * - :ref:`castshadow<body-light-castshadow>`
              - :ref:`active<body-light-active>`
              - :ref:`pos<body-light-pos>`
              - :ref:`dir<body-light-dir>`
            * - :ref:`bulbradius<body-light-bulbradius>`
              - :ref:`intensity<body-light-intensity>`
              - :ref:`range<body-light-range>`
              - :ref:`attenuation<body-light-attenuation>`
            * - :ref:`cutoff<body-light-cutoff>`
              - :ref:`exponent<body-light-exponent>`
              - :ref:`ambient<body-light-ambient>`
              - :ref:`diffuse<body-light-diffuse>`
            * - :ref:`specular<body-light-specular>`
              - :ref:`mode<body-light-mode>`
              - :ref:`target<body-light-target>`
              - :ref:`texture<body-light-texture>`

      .. dropdown:: :ref:`plugin<body-plugin>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`plugin<body-plugin-plugin>`
              - :ref:`instance<body-plugin-instance>`
              -
              -

         .. dropdown:: :ref:`config<plugin-config>`

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`key<plugin-config-key>`
                 - :ref:`value<plugin-config-value>`
                 -
                 -

      .. dropdown:: :ref:`composite<body-composite>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`prefix<body-composite-prefix>`
              - :ref:`type<body-composite-type>`
              - :ref:`count<body-composite-count>`
              - :ref:`offset<body-composite-offset>`
            * - :ref:`vertex<body-composite-vertex>`
              - :ref:`initial<body-composite-initial>`
              - :ref:`curve<body-composite-curve>`
              - :ref:`size<body-composite-size>`
            * - :ref:`quat<body-composite-quat>`
              -
              -
              -

         .. dropdown:: :ref:`joint<composite-joint>`

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`kind<composite-joint-kind>`
                 - :ref:`group<composite-joint-group>`
                 - :ref:`stiffness<composite-joint-stiffness>`
                 - :ref:`damping<composite-joint-damping>`
               * - :ref:`armature<composite-joint-armature>`
                 - :ref:`solreffix<composite-joint-solreffix>`
                 - :ref:`solimpfix<composite-joint-solimpfix>`
                 - :ref:`type<composite-joint-type>`
               * - :ref:`axis<composite-joint-axis>`
                 - :ref:`limited<composite-joint-limited>`
                 - :ref:`range<composite-joint-range>`
                 - :ref:`margin<composite-joint-margin>`
               * - :ref:`solreflimit<composite-joint-solreflimit>`
                 - :ref:`solimplimit<composite-joint-solimplimit>`
                 - :ref:`frictionloss<composite-joint-frictionloss>`
                 - :ref:`solreffriction<composite-joint-solreffriction>`
               * - :ref:`solimpfriction<composite-joint-solimpfriction>`
                 -
                 -
                 -

         .. dropdown:: :ref:`skin<composite-skin>`
            :icon: dot

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`texcoord<composite-skin-texcoord>`
                 - :ref:`material<composite-skin-material>`
                 - :ref:`group<composite-skin-group>`
                 - :ref:`rgba<composite-skin-rgba>`
               * - :ref:`inflate<composite-skin-inflate>`
                 - :ref:`subgrid<composite-skin-subgrid>`
                 -
                 -

         .. dropdown:: :ref:`geom<composite-geom>`
            :icon: dot

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`type<composite-geom-type>`
                 - :ref:`contype<composite-geom-contype>`
                 - :ref:`conaffinity<composite-geom-conaffinity>`
                 - :ref:`condim<composite-geom-condim>`
               * - :ref:`group<composite-geom-group>`
                 - :ref:`priority<composite-geom-priority>`
                 - :ref:`size<composite-geom-size>`
                 - :ref:`material<composite-geom-material>`
               * - :ref:`rgba<composite-geom-rgba>`
                 - :ref:`friction<composite-geom-friction>`
                 - :ref:`mass<composite-geom-mass>`
                 - :ref:`density<composite-geom-density>`
               * - :ref:`solmix<composite-geom-solmix>`
                 - :ref:`solref<composite-geom-solref>`
                 - :ref:`solimp<composite-geom-solimp>`
                 - :ref:`margin<composite-geom-margin>`
               * - :ref:`gap<composite-geom-gap>`
                 -
                 -
                 -

         .. dropdown:: :ref:`site<composite-site>`
            :icon: dot

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`group<composite-site-group>`
                 - :ref:`size<composite-site-size>`
                 - :ref:`material<composite-site-material>`
                 - :ref:`rgba<composite-site-rgba>`

         .. dropdown:: :ref:`plugin<composite-plugin>`

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`plugin<composite-plugin-plugin>`
                 - :ref:`instance<composite-plugin-instance>`
                 -
                 -

            .. dropdown:: :ref:`config<plugin-config>`

               .. list-table::
                  :class: mjcf-attributes
                  :widths: 25 25 25 25

                  * - :ref:`key<plugin-config-key>`
                    - :ref:`value<plugin-config-value>`
                    -
                    -

      .. dropdown:: :ref:`flexcomp<body-flexcomp>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<body-flexcomp-name>`
              - :ref:`type<body-flexcomp-type>`
              - :ref:`group<body-flexcomp-group>`
              - :ref:`dim<body-flexcomp-dim>`
            * - :ref:`dof<body-flexcomp-dof>`
              - :ref:`count<body-flexcomp-count>`
              - :ref:`spacing<body-flexcomp-spacing>`
              - :ref:`radius<body-flexcomp-radius>`
            * - :ref:`rigid<body-flexcomp-rigid>`
              - :ref:`mass<body-flexcomp-mass>`
              - :ref:`inertiabox<body-flexcomp-inertiabox>`
              - :ref:`scale<body-flexcomp-scale>`
            * - :ref:`file<body-flexcomp-file>`
              - :ref:`point<body-flexcomp-point>`
              - :ref:`element<body-flexcomp-element>`
              - :ref:`texcoord<body-flexcomp-texcoord>`
            * - :ref:`material<body-flexcomp-material>`
              - :ref:`rgba<body-flexcomp-rgba>`
              - :ref:`flatskin<body-flexcomp-flatskin>`
              - :ref:`pos<body-flexcomp-pos>`
            * - :ref:`quat<body-flexcomp-quat>`
              - :ref:`axisangle<body-flexcomp-axisangle>`
              - :ref:`xyaxes<body-flexcomp-xyaxes>`
              - :ref:`zaxis<body-flexcomp-zaxis>`
            * - :ref:`euler<body-flexcomp-euler>`
              - :ref:`origin<body-flexcomp-origin>`
              -
              -

         .. dropdown:: :ref:`edge<flexcomp-edge>`
            :icon: dot

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`equality<flexcomp-edge-equality>`
                 - :ref:`solref<flexcomp-edge-solref>`
                 - :ref:`solimp<flexcomp-edge-solimp>`
                 - :ref:`stiffness<flexcomp-edge-stiffness>`
               * - :ref:`damping<flexcomp-edge-damping>`
                 -
                 -
                 -

         .. dropdown:: :ref:`elasticity<flexcomp-elasticity>`
            :icon: dot

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`young<flexcomp-elasticity-young>`
                 - :ref:`poisson<flexcomp-elasticity-poisson>`
                 - :ref:`damping<flexcomp-elasticity-damping>`
                 - :ref:`thickness<flexcomp-elasticity-thickness>`
               * - :ref:`elastic2d<flexcomp-elasticity-elastic2d>`
                 -
                 -
                 -

         .. dropdown:: :ref:`contact<flexcomp-contact>`
            :icon: dot

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`contype<flexcomp-contact-contype>`
                 - :ref:`conaffinity<flexcomp-contact-conaffinity>`
                 - :ref:`condim<flexcomp-contact-condim>`
                 - :ref:`priority<flexcomp-contact-priority>`
               * - :ref:`friction<flexcomp-contact-friction>`
                 - :ref:`solmix<flexcomp-contact-solmix>`
                 - :ref:`solref<flexcomp-contact-solref>`
                 - :ref:`solimp<flexcomp-contact-solimp>`
               * - :ref:`margin<flexcomp-contact-margin>`
                 - :ref:`gap<flexcomp-contact-gap>`
                 - :ref:`internal<flexcomp-contact-internal>`
                 - :ref:`selfcollide<flexcomp-contact-selfcollide>`
               * - :ref:`activelayers<flexcomp-contact-activelayers>`
                 - :ref:`vertcollide<flexcomp-contact-vertcollide>`
                 - :ref:`passive<flexcomp-contact-passive>`
                 -

         .. dropdown:: :ref:`pin<flexcomp-pin>`

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`id<flexcomp-pin-id>`
                 - :ref:`range<flexcomp-pin-range>`
                 - :ref:`grid<flexcomp-pin-grid>`
                 - :ref:`gridrange<flexcomp-pin-gridrange>`

         .. dropdown:: :ref:`plugin<flexcomp-plugin>`

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`plugin<flexcomp-plugin-plugin>`
                 - :ref:`instance<flexcomp-plugin-instance>`
                 -
                 -

            .. dropdown:: :ref:`config<plugin-config>`

               .. list-table::
                  :class: mjcf-attributes
                  :widths: 25 25 25 25

                  * - :ref:`key<plugin-config-key>`
                    - :ref:`value<plugin-config-value>`
                    -
                    -

   .. dropdown:: :ref:`deformable<deformable>`


      .. dropdown:: :ref:`flex<deformable-flex>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<deformable-flex-name>`
              - :ref:`group<deformable-flex-group>`
              - :ref:`dim<deformable-flex-dim>`
              - :ref:`radius<deformable-flex-radius>`
            * - :ref:`material<deformable-flex-material>`
              - :ref:`rgba<deformable-flex-rgba>`
              - :ref:`flatskin<deformable-flex-flatskin>`
              - :ref:`body<deformable-flex-body>`
            * - :ref:`vertex<deformable-flex-vertex>`
              - :ref:`element<deformable-flex-element>`
              - :ref:`texcoord<deformable-flex-texcoord>`
              - :ref:`elemtexcoord<deformable-flex-elemtexcoord>`
            * - :ref:`node<deformable-flex-node>`
              -
              -
              -

         .. dropdown:: :ref:`contact<flex-contact>`
            :icon: dot

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`contype<flex-contact-contype>`
                 - :ref:`conaffinity<flex-contact-conaffinity>`
                 - :ref:`condim<flex-contact-condim>`
                 - :ref:`priority<flex-contact-priority>`
               * - :ref:`friction<flex-contact-friction>`
                 - :ref:`solmix<flex-contact-solmix>`
                 - :ref:`solref<flex-contact-solref>`
                 - :ref:`solimp<flex-contact-solimp>`
               * - :ref:`margin<flex-contact-margin>`
                 - :ref:`gap<flex-contact-gap>`
                 - :ref:`internal<flex-contact-internal>`
                 - :ref:`selfcollide<flex-contact-selfcollide>`
               * - :ref:`activelayers<flex-contact-activelayers>`
                 - :ref:`vertcollide<flex-contact-vertcollide>`
                 - :ref:`passive<flex-contact-passive>`
                 -

         .. dropdown:: :ref:`edge<flex-edge>`
            :icon: dot

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`stiffness<flex-edge-stiffness>`
                 - :ref:`damping<flex-edge-damping>`
                 -
                 -

         .. dropdown:: :ref:`elasticity<flex-elasticity>`
            :icon: dot

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`young<flex-elasticity-young>`
                 - :ref:`poisson<flex-elasticity-poisson>`
                 - :ref:`damping<flex-elasticity-damping>`
                 - :ref:`thickness<flex-elasticity-thickness>`
               * - :ref:`elastic2d<flex-elasticity-elastic2d>`
                 -
                 -
                 -

      .. dropdown:: :ref:`skin<deformable-skin>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<deformable-skin-name>`
              - :ref:`file<deformable-skin-file>`
              - :ref:`material<deformable-skin-material>`
              - :ref:`rgba<deformable-skin-rgba>`
            * - :ref:`inflate<deformable-skin-inflate>`
              - :ref:`vertex<deformable-skin-vertex>`
              - :ref:`texcoord<deformable-skin-texcoord>`
              - :ref:`face<deformable-skin-face>`
            * - :ref:`group<deformable-skin-group>`
              -
              -
              -

         .. dropdown:: :ref:`bone<skin-bone>`

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`body<skin-bone-body>`
                 - :ref:`bindpos<skin-bone-bindpos>`
                 - :ref:`bindquat<skin-bone-bindquat>`
                 - :ref:`vertid<skin-bone-vertid>`
               * - :ref:`vertweight<skin-bone-vertweight>`
                 -
                 -
                 -

   .. dropdown:: :ref:`contact<contact>`


      .. dropdown:: :ref:`pair<contact-pair>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<contact-pair-name>`
              - :ref:`class<contact-pair-class>`
              - :ref:`geom1<contact-pair-geom1>`
              - :ref:`geom2<contact-pair-geom2>`
            * - :ref:`condim<contact-pair-condim>`
              - :ref:`friction<contact-pair-friction>`
              - :ref:`solref<contact-pair-solref>`
              - :ref:`solreffriction<contact-pair-solreffriction>`
            * - :ref:`solimp<contact-pair-solimp>`
              - :ref:`gap<contact-pair-gap>`
              - :ref:`margin<contact-pair-margin>`
              -

      .. dropdown:: :ref:`exclude<contact-exclude>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<contact-exclude-name>`
              - :ref:`body1<contact-exclude-body1>`
              - :ref:`body2<contact-exclude-body2>`
              -

   .. dropdown:: :ref:`equality<equality>`


      .. dropdown:: :ref:`connect<equality-connect>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<equality-connect-name>`
              - :ref:`class<equality-connect-class>`
              - :ref:`body1<equality-connect-body1>`
              - :ref:`body2<equality-connect-body2>`
            * - :ref:`anchor<equality-connect-anchor>`
              - :ref:`site1<equality-connect-site1>`
              - :ref:`site2<equality-connect-site2>`
              - :ref:`active<equality-connect-active>`
            * - :ref:`solref<equality-connect-solref>`
              - :ref:`solimp<equality-connect-solimp>`
              -
              -

      .. dropdown:: :ref:`weld<equality-weld>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<equality-weld-name>`
              - :ref:`class<equality-weld-class>`
              - :ref:`body1<equality-weld-body1>`
              - :ref:`body2<equality-weld-body2>`
            * - :ref:`relpose<equality-weld-relpose>`
              - :ref:`anchor<equality-weld-anchor>`
              - :ref:`site1<equality-weld-site1>`
              - :ref:`site2<equality-weld-site2>`
            * - :ref:`active<equality-weld-active>`
              - :ref:`solref<equality-weld-solref>`
              - :ref:`solimp<equality-weld-solimp>`
              - :ref:`torquescale<equality-weld-torquescale>`

      .. dropdown:: :ref:`joint<equality-joint>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<equality-joint-name>`
              - :ref:`class<equality-joint-class>`
              - :ref:`joint1<equality-joint-joint1>`
              - :ref:`joint2<equality-joint-joint2>`
            * - :ref:`polycoef<equality-joint-polycoef>`
              - :ref:`active<equality-joint-active>`
              - :ref:`solref<equality-joint-solref>`
              - :ref:`solimp<equality-joint-solimp>`

      .. dropdown:: :ref:`tendon<equality-tendon>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<equality-tendon-name>`
              - :ref:`class<equality-tendon-class>`
              - :ref:`tendon1<equality-tendon-tendon1>`
              - :ref:`tendon2<equality-tendon-tendon2>`
            * - :ref:`polycoef<equality-tendon-polycoef>`
              - :ref:`active<equality-tendon-active>`
              - :ref:`solref<equality-tendon-solref>`
              - :ref:`solimp<equality-tendon-solimp>`

      .. dropdown:: :ref:`flex<equality-flex>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<equality-flex-name>`
              - :ref:`class<equality-flex-class>`
              - :ref:`flex<equality-flex-flex>`
              - :ref:`active<equality-flex-active>`
            * - :ref:`solref<equality-flex-solref>`
              - :ref:`solimp<equality-flex-solimp>`
              -
              -

   .. dropdown:: :ref:`tendon<tendon>`


      .. dropdown:: :ref:`spatial<tendon-spatial>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<tendon-spatial-name>`
              - :ref:`class<tendon-spatial-class>`
              - :ref:`group<tendon-spatial-group>`
              - :ref:`limited<tendon-spatial-limited>`
            * - :ref:`actuatorfrclimited<tendon-spatial-actuatorfrclimited>`
              - :ref:`range<tendon-spatial-range>`
              - :ref:`actuatorfrcrange<tendon-spatial-actuatorfrcrange>`
              - :ref:`solreflimit<tendon-spatial-solreflimit>`
            * - :ref:`solimplimit<tendon-spatial-solimplimit>`
              - :ref:`solreffriction<tendon-spatial-solreffriction>`
              - :ref:`solimpfriction<tendon-spatial-solimpfriction>`
              - :ref:`frictionloss<tendon-spatial-frictionloss>`
            * - :ref:`springlength<tendon-spatial-springlength>`
              - :ref:`width<tendon-spatial-width>`
              - :ref:`material<tendon-spatial-material>`
              - :ref:`margin<tendon-spatial-margin>`
            * - :ref:`stiffness<tendon-spatial-stiffness>`
              - :ref:`damping<tendon-spatial-damping>`
              - :ref:`armature<tendon-spatial-armature>`
              - :ref:`rgba<tendon-spatial-rgba>`
            * - :ref:`user<tendon-spatial-user>`
              -
              -
              -

         .. dropdown:: :ref:`site<spatial-site>`

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`site<spatial-site-site>`
                 -
                 -
                 -

         .. dropdown:: :ref:`geom<spatial-geom>`

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`geom<spatial-geom-geom>`
                 - :ref:`sidesite<spatial-geom-sidesite>`
                 -
                 -

         .. dropdown:: :ref:`pulley<spatial-pulley>`

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`divisor<spatial-pulley-divisor>`
                 -
                 -
                 -

      .. dropdown:: :ref:`fixed<tendon-fixed>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<tendon-fixed-name>`
              - :ref:`class<tendon-fixed-class>`
              - :ref:`group<tendon-fixed-group>`
              - :ref:`limited<tendon-fixed-limited>`
            * - :ref:`actuatorfrclimited<tendon-fixed-actuatorfrclimited>`
              - :ref:`range<tendon-fixed-range>`
              - :ref:`actuatorfrcrange<tendon-fixed-actuatorfrcrange>`
              - :ref:`solreflimit<tendon-fixed-solreflimit>`
            * - :ref:`solimplimit<tendon-fixed-solimplimit>`
              - :ref:`solreffriction<tendon-fixed-solreffriction>`
              - :ref:`solimpfriction<tendon-fixed-solimpfriction>`
              - :ref:`frictionloss<tendon-fixed-frictionloss>`
            * - :ref:`springlength<tendon-fixed-springlength>`
              - :ref:`margin<tendon-fixed-margin>`
              - :ref:`stiffness<tendon-fixed-stiffness>`
              - :ref:`damping<tendon-fixed-damping>`
            * - :ref:`armature<tendon-fixed-armature>`
              - :ref:`user<tendon-fixed-user>`
              -
              -

         .. dropdown:: :ref:`joint<fixed-joint>`

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`joint<fixed-joint-joint>`
                 - :ref:`coef<fixed-joint-coef>`
                 -
                 -

   .. dropdown:: :ref:`actuator<actuator>`


      .. dropdown:: :ref:`general<actuator-general>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<actuator-general-name>`
              - :ref:`class<actuator-general-class>`
              - :ref:`group<actuator-general-group>`
              - :ref:`ctrllimited<actuator-general-ctrllimited>`
            * - :ref:`forcelimited<actuator-general-forcelimited>`
              - :ref:`actlimited<actuator-general-actlimited>`
              - :ref:`ctrlrange<actuator-general-ctrlrange>`
              - :ref:`forcerange<actuator-general-forcerange>`
            * - :ref:`actrange<actuator-general-actrange>`
              - :ref:`lengthrange<actuator-general-lengthrange>`
              - :ref:`gear<actuator-general-gear>`
              - :ref:`cranklength<actuator-general-cranklength>`
            * - :ref:`user<actuator-general-user>`
              - :ref:`joint<actuator-general-joint>`
              - :ref:`jointinparent<actuator-general-jointinparent>`
              - :ref:`tendon<actuator-general-tendon>`
            * - :ref:`slidersite<actuator-general-slidersite>`
              - :ref:`cranksite<actuator-general-cranksite>`
              - :ref:`site<actuator-general-site>`
              - :ref:`refsite<actuator-general-refsite>`
            * - :ref:`body<actuator-general-body>`
              - :ref:`actdim<actuator-general-actdim>`
              - :ref:`dyntype<actuator-general-dyntype>`
              - :ref:`gaintype<actuator-general-gaintype>`
            * - :ref:`biastype<actuator-general-biastype>`
              - :ref:`dynprm<actuator-general-dynprm>`
              - :ref:`gainprm<actuator-general-gainprm>`
              - :ref:`biasprm<actuator-general-biasprm>`
            * - :ref:`actearly<actuator-general-actearly>`
              -
              -
              -

      .. dropdown:: :ref:`motor<actuator-motor>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<actuator-motor-name>`
              - :ref:`class<actuator-motor-class>`
              - :ref:`group<actuator-motor-group>`
              - :ref:`ctrllimited<actuator-motor-ctrllimited>`
            * - :ref:`forcelimited<actuator-motor-forcelimited>`
              - :ref:`ctrlrange<actuator-motor-ctrlrange>`
              - :ref:`forcerange<actuator-motor-forcerange>`
              - :ref:`lengthrange<actuator-motor-lengthrange>`
            * - :ref:`gear<actuator-motor-gear>`
              - :ref:`cranklength<actuator-motor-cranklength>`
              - :ref:`user<actuator-motor-user>`
              - :ref:`joint<actuator-motor-joint>`
            * - :ref:`jointinparent<actuator-motor-jointinparent>`
              - :ref:`tendon<actuator-motor-tendon>`
              - :ref:`slidersite<actuator-motor-slidersite>`
              - :ref:`cranksite<actuator-motor-cranksite>`
            * - :ref:`site<actuator-motor-site>`
              - :ref:`refsite<actuator-motor-refsite>`
              -
              -

      .. dropdown:: :ref:`position<actuator-position>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<actuator-position-name>`
              - :ref:`class<actuator-position-class>`
              - :ref:`group<actuator-position-group>`
              - :ref:`ctrllimited<actuator-position-ctrllimited>`
            * - :ref:`forcelimited<actuator-position-forcelimited>`
              - :ref:`ctrlrange<actuator-position-ctrlrange>`
              - :ref:`inheritrange<actuator-position-inheritrange>`
              - :ref:`forcerange<actuator-position-forcerange>`
            * - :ref:`lengthrange<actuator-position-lengthrange>`
              - :ref:`gear<actuator-position-gear>`
              - :ref:`cranklength<actuator-position-cranklength>`
              - :ref:`user<actuator-position-user>`
            * - :ref:`joint<actuator-position-joint>`
              - :ref:`jointinparent<actuator-position-jointinparent>`
              - :ref:`tendon<actuator-position-tendon>`
              - :ref:`slidersite<actuator-position-slidersite>`
            * - :ref:`cranksite<actuator-position-cranksite>`
              - :ref:`site<actuator-position-site>`
              - :ref:`refsite<actuator-position-refsite>`
              - :ref:`kp<actuator-position-kp>`
            * - :ref:`kv<actuator-position-kv>`
              - :ref:`dampratio<actuator-position-dampratio>`
              - :ref:`timeconst<actuator-position-timeconst>`
              -

      .. dropdown:: :ref:`velocity<actuator-velocity>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<actuator-velocity-name>`
              - :ref:`class<actuator-velocity-class>`
              - :ref:`group<actuator-velocity-group>`
              - :ref:`ctrllimited<actuator-velocity-ctrllimited>`
            * - :ref:`forcelimited<actuator-velocity-forcelimited>`
              - :ref:`ctrlrange<actuator-velocity-ctrlrange>`
              - :ref:`forcerange<actuator-velocity-forcerange>`
              - :ref:`lengthrange<actuator-velocity-lengthrange>`
            * - :ref:`gear<actuator-velocity-gear>`
              - :ref:`cranklength<actuator-velocity-cranklength>`
              - :ref:`user<actuator-velocity-user>`
              - :ref:`joint<actuator-velocity-joint>`
            * - :ref:`jointinparent<actuator-velocity-jointinparent>`
              - :ref:`tendon<actuator-velocity-tendon>`
              - :ref:`slidersite<actuator-velocity-slidersite>`
              - :ref:`cranksite<actuator-velocity-cranksite>`
            * - :ref:`site<actuator-velocity-site>`
              - :ref:`refsite<actuator-velocity-refsite>`
              - :ref:`kv<actuator-velocity-kv>`
              -

      .. dropdown:: :ref:`intvelocity<actuator-intvelocity>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<actuator-intvelocity-name>`
              - :ref:`class<actuator-intvelocity-class>`
              - :ref:`group<actuator-intvelocity-group>`
              - :ref:`ctrllimited<actuator-intvelocity-ctrllimited>`
            * - :ref:`forcelimited<actuator-intvelocity-forcelimited>`
              - :ref:`ctrlrange<actuator-intvelocity-ctrlrange>`
              - :ref:`forcerange<actuator-intvelocity-forcerange>`
              - :ref:`actrange<actuator-intvelocity-actrange>`
            * - :ref:`inheritrange<actuator-intvelocity-inheritrange>`
              - :ref:`lengthrange<actuator-intvelocity-lengthrange>`
              - :ref:`gear<actuator-intvelocity-gear>`
              - :ref:`cranklength<actuator-intvelocity-cranklength>`
            * - :ref:`user<actuator-intvelocity-user>`
              - :ref:`joint<actuator-intvelocity-joint>`
              - :ref:`jointinparent<actuator-intvelocity-jointinparent>`
              - :ref:`tendon<actuator-intvelocity-tendon>`
            * - :ref:`slidersite<actuator-intvelocity-slidersite>`
              - :ref:`cranksite<actuator-intvelocity-cranksite>`
              - :ref:`site<actuator-intvelocity-site>`
              - :ref:`refsite<actuator-intvelocity-refsite>`
            * - :ref:`kp<actuator-intvelocity-kp>`
              - :ref:`kv<actuator-intvelocity-kv>`
              - :ref:`dampratio<actuator-intvelocity-dampratio>`
              -

      .. dropdown:: :ref:`damper<actuator-damper>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<actuator-damper-name>`
              - :ref:`class<actuator-damper-class>`
              - :ref:`group<actuator-damper-group>`
              - :ref:`forcelimited<actuator-damper-forcelimited>`
            * - :ref:`ctrlrange<actuator-damper-ctrlrange>`
              - :ref:`forcerange<actuator-damper-forcerange>`
              - :ref:`lengthrange<actuator-damper-lengthrange>`
              - :ref:`gear<actuator-damper-gear>`
            * - :ref:`cranklength<actuator-damper-cranklength>`
              - :ref:`user<actuator-damper-user>`
              - :ref:`joint<actuator-damper-joint>`
              - :ref:`jointinparent<actuator-damper-jointinparent>`
            * - :ref:`tendon<actuator-damper-tendon>`
              - :ref:`slidersite<actuator-damper-slidersite>`
              - :ref:`cranksite<actuator-damper-cranksite>`
              - :ref:`site<actuator-damper-site>`
            * - :ref:`refsite<actuator-damper-refsite>`
              - :ref:`kv<actuator-damper-kv>`
              -
              -

      .. dropdown:: :ref:`cylinder<actuator-cylinder>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<actuator-cylinder-name>`
              - :ref:`class<actuator-cylinder-class>`
              - :ref:`group<actuator-cylinder-group>`
              - :ref:`ctrllimited<actuator-cylinder-ctrllimited>`
            * - :ref:`forcelimited<actuator-cylinder-forcelimited>`
              - :ref:`ctrlrange<actuator-cylinder-ctrlrange>`
              - :ref:`forcerange<actuator-cylinder-forcerange>`
              - :ref:`lengthrange<actuator-cylinder-lengthrange>`
            * - :ref:`gear<actuator-cylinder-gear>`
              - :ref:`cranklength<actuator-cylinder-cranklength>`
              - :ref:`user<actuator-cylinder-user>`
              - :ref:`joint<actuator-cylinder-joint>`
            * - :ref:`jointinparent<actuator-cylinder-jointinparent>`
              - :ref:`tendon<actuator-cylinder-tendon>`
              - :ref:`slidersite<actuator-cylinder-slidersite>`
              - :ref:`cranksite<actuator-cylinder-cranksite>`
            * - :ref:`site<actuator-cylinder-site>`
              - :ref:`refsite<actuator-cylinder-refsite>`
              - :ref:`timeconst<actuator-cylinder-timeconst>`
              - :ref:`area<actuator-cylinder-area>`
            * - :ref:`diameter<actuator-cylinder-diameter>`
              - :ref:`bias<actuator-cylinder-bias>`
              -
              -

      .. dropdown:: :ref:`muscle<actuator-muscle>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<actuator-muscle-name>`
              - :ref:`class<actuator-muscle-class>`
              - :ref:`group<actuator-muscle-group>`
              - :ref:`ctrllimited<actuator-muscle-ctrllimited>`
            * - :ref:`forcelimited<actuator-muscle-forcelimited>`
              - :ref:`ctrlrange<actuator-muscle-ctrlrange>`
              - :ref:`forcerange<actuator-muscle-forcerange>`
              - :ref:`lengthrange<actuator-muscle-lengthrange>`
            * - :ref:`gear<actuator-muscle-gear>`
              - :ref:`cranklength<actuator-muscle-cranklength>`
              - :ref:`user<actuator-muscle-user>`
              - :ref:`joint<actuator-muscle-joint>`
            * - :ref:`jointinparent<actuator-muscle-jointinparent>`
              - :ref:`tendon<actuator-muscle-tendon>`
              - :ref:`slidersite<actuator-muscle-slidersite>`
              - :ref:`cranksite<actuator-muscle-cranksite>`
            * - :ref:`timeconst<actuator-muscle-timeconst>`
              - :ref:`tausmooth<actuator-muscle-tausmooth>`
              - :ref:`range<actuator-muscle-range>`
              - :ref:`force<actuator-muscle-force>`
            * - :ref:`scale<actuator-muscle-scale>`
              - :ref:`lmin<actuator-muscle-lmin>`
              - :ref:`lmax<actuator-muscle-lmax>`
              - :ref:`vmax<actuator-muscle-vmax>`
            * - :ref:`fpmax<actuator-muscle-fpmax>`
              - :ref:`fvmax<actuator-muscle-fvmax>`
              -
              -

      .. dropdown:: :ref:`adhesion<actuator-adhesion>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<actuator-adhesion-name>`
              - :ref:`class<actuator-adhesion-class>`
              - :ref:`group<actuator-adhesion-group>`
              - :ref:`forcelimited<actuator-adhesion-forcelimited>`
            * - :ref:`ctrlrange<actuator-adhesion-ctrlrange>`
              - :ref:`forcerange<actuator-adhesion-forcerange>`
              - :ref:`user<actuator-adhesion-user>`
              - :ref:`body<actuator-adhesion-body>`
            * - :ref:`gain<actuator-adhesion-gain>`
              -
              -
              -

      .. dropdown:: :ref:`plugin<actuator-plugin>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<actuator-plugin-name>`
              - :ref:`class<actuator-plugin-class>`
              - :ref:`plugin<actuator-plugin-plugin>`
              - :ref:`instance<actuator-plugin-instance>`
            * - :ref:`group<actuator-plugin-group>`
              - :ref:`ctrllimited<actuator-plugin-ctrllimited>`
              - :ref:`forcelimited<actuator-plugin-forcelimited>`
              - :ref:`actlimited<actuator-plugin-actlimited>`
            * - :ref:`ctrlrange<actuator-plugin-ctrlrange>`
              - :ref:`forcerange<actuator-plugin-forcerange>`
              - :ref:`actrange<actuator-plugin-actrange>`
              - :ref:`lengthrange<actuator-plugin-lengthrange>`
            * - :ref:`gear<actuator-plugin-gear>`
              - :ref:`cranklength<actuator-plugin-cranklength>`
              - :ref:`joint<actuator-plugin-joint>`
              - :ref:`jointinparent<actuator-plugin-jointinparent>`
            * - :ref:`site<actuator-plugin-site>`
              - :ref:`actdim<actuator-plugin-actdim>`
              - :ref:`dyntype<actuator-plugin-dyntype>`
              - :ref:`dynprm<actuator-plugin-dynprm>`
            * - :ref:`tendon<actuator-plugin-tendon>`
              - :ref:`cranksite<actuator-plugin-cranksite>`
              - :ref:`slidersite<actuator-plugin-slidersite>`
              - :ref:`user<actuator-plugin-user>`
            * - :ref:`actearly<actuator-plugin-actearly>`
              -
              -
              -

         .. dropdown:: :ref:`config<plugin-config>`

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`key<plugin-config-key>`
                 - :ref:`value<plugin-config-value>`
                 -
                 -

   .. dropdown:: :ref:`sensor<sensor>`


      .. dropdown:: :ref:`touch<sensor-touch>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-touch-name>`
              - :ref:`site<sensor-touch-site>`
              - :ref:`cutoff<sensor-touch-cutoff>`
              - :ref:`noise<sensor-touch-noise>`
            * - :ref:`user<sensor-touch-user>`
              -
              -
              -

      .. dropdown:: :ref:`accelerometer<sensor-accelerometer>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-accelerometer-name>`
              - :ref:`site<sensor-accelerometer-site>`
              - :ref:`cutoff<sensor-accelerometer-cutoff>`
              - :ref:`noise<sensor-accelerometer-noise>`
            * - :ref:`user<sensor-accelerometer-user>`
              -
              -
              -

      .. dropdown:: :ref:`velocimeter<sensor-velocimeter>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-velocimeter-name>`
              - :ref:`site<sensor-velocimeter-site>`
              - :ref:`cutoff<sensor-velocimeter-cutoff>`
              - :ref:`noise<sensor-velocimeter-noise>`
            * - :ref:`user<sensor-velocimeter-user>`
              -
              -
              -

      .. dropdown:: :ref:`gyro<sensor-gyro>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-gyro-name>`
              - :ref:`site<sensor-gyro-site>`
              - :ref:`cutoff<sensor-gyro-cutoff>`
              - :ref:`noise<sensor-gyro-noise>`
            * - :ref:`user<sensor-gyro-user>`
              -
              -
              -

      .. dropdown:: :ref:`force<sensor-force>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-force-name>`
              - :ref:`site<sensor-force-site>`
              - :ref:`cutoff<sensor-force-cutoff>`
              - :ref:`noise<sensor-force-noise>`
            * - :ref:`user<sensor-force-user>`
              -
              -
              -

      .. dropdown:: :ref:`torque<sensor-torque>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-torque-name>`
              - :ref:`site<sensor-torque-site>`
              - :ref:`cutoff<sensor-torque-cutoff>`
              - :ref:`noise<sensor-torque-noise>`
            * - :ref:`user<sensor-torque-user>`
              -
              -
              -

      .. dropdown:: :ref:`magnetometer<sensor-magnetometer>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-magnetometer-name>`
              - :ref:`site<sensor-magnetometer-site>`
              - :ref:`cutoff<sensor-magnetometer-cutoff>`
              - :ref:`noise<sensor-magnetometer-noise>`
            * - :ref:`user<sensor-magnetometer-user>`
              -
              -
              -

      .. dropdown:: :ref:`camprojection<sensor-camprojection>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-camprojection-name>`
              - :ref:`site<sensor-camprojection-site>`
              - :ref:`camera<sensor-camprojection-camera>`
              - :ref:`cutoff<sensor-camprojection-cutoff>`
            * - :ref:`noise<sensor-camprojection-noise>`
              - :ref:`user<sensor-camprojection-user>`
              -
              -

      .. dropdown:: :ref:`rangefinder<sensor-rangefinder>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-rangefinder-name>`
              - :ref:`site<sensor-rangefinder-site>`
              - :ref:`camera<sensor-rangefinder-camera>`
              - :ref:`data<sensor-rangefinder-data>`
            * - :ref:`cutoff<sensor-rangefinder-cutoff>`
              - :ref:`noise<sensor-rangefinder-noise>`
              - :ref:`user<sensor-rangefinder-user>`
              -

      .. dropdown:: :ref:`jointpos<sensor-jointpos>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-jointpos-name>`
              - :ref:`joint<sensor-jointpos-joint>`
              - :ref:`cutoff<sensor-jointpos-cutoff>`
              - :ref:`noise<sensor-jointpos-noise>`
            * - :ref:`user<sensor-jointpos-user>`
              -
              -
              -

      .. dropdown:: :ref:`jointvel<sensor-jointvel>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-jointvel-name>`
              - :ref:`joint<sensor-jointvel-joint>`
              - :ref:`cutoff<sensor-jointvel-cutoff>`
              - :ref:`noise<sensor-jointvel-noise>`
            * - :ref:`user<sensor-jointvel-user>`
              -
              -
              -

      .. dropdown:: :ref:`tendonpos<sensor-tendonpos>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-tendonpos-name>`
              - :ref:`tendon<sensor-tendonpos-tendon>`
              - :ref:`cutoff<sensor-tendonpos-cutoff>`
              - :ref:`noise<sensor-tendonpos-noise>`
            * - :ref:`user<sensor-tendonpos-user>`
              -
              -
              -

      .. dropdown:: :ref:`tendonvel<sensor-tendonvel>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-tendonvel-name>`
              - :ref:`tendon<sensor-tendonvel-tendon>`
              - :ref:`cutoff<sensor-tendonvel-cutoff>`
              - :ref:`noise<sensor-tendonvel-noise>`
            * - :ref:`user<sensor-tendonvel-user>`
              -
              -
              -

      .. dropdown:: :ref:`actuatorpos<sensor-actuatorpos>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-actuatorpos-name>`
              - :ref:`actuator<sensor-actuatorpos-actuator>`
              - :ref:`cutoff<sensor-actuatorpos-cutoff>`
              - :ref:`noise<sensor-actuatorpos-noise>`
            * - :ref:`user<sensor-actuatorpos-user>`
              -
              -
              -

      .. dropdown:: :ref:`actuatorvel<sensor-actuatorvel>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-actuatorvel-name>`
              - :ref:`actuator<sensor-actuatorvel-actuator>`
              - :ref:`cutoff<sensor-actuatorvel-cutoff>`
              - :ref:`noise<sensor-actuatorvel-noise>`
            * - :ref:`user<sensor-actuatorvel-user>`
              -
              -
              -

      .. dropdown:: :ref:`actuatorfrc<sensor-actuatorfrc>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-actuatorfrc-name>`
              - :ref:`actuator<sensor-actuatorfrc-actuator>`
              - :ref:`cutoff<sensor-actuatorfrc-cutoff>`
              - :ref:`noise<sensor-actuatorfrc-noise>`
            * - :ref:`user<sensor-actuatorfrc-user>`
              -
              -
              -

      .. dropdown:: :ref:`jointactuatorfrc<sensor-jointactuatorfrc>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-jointactuatorfrc-name>`
              - :ref:`joint<sensor-jointactuatorfrc-joint>`
              - :ref:`cutoff<sensor-jointactuatorfrc-cutoff>`
              - :ref:`noise<sensor-jointactuatorfrc-noise>`
            * - :ref:`user<sensor-jointactuatorfrc-user>`
              -
              -
              -

      .. dropdown:: :ref:`tendonactuatorfrc<sensor-tendonactuatorfrc>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-tendonactuatorfrc-name>`
              - :ref:`tendon<sensor-tendonactuatorfrc-tendon>`
              - :ref:`cutoff<sensor-tendonactuatorfrc-cutoff>`
              - :ref:`noise<sensor-tendonactuatorfrc-noise>`
            * - :ref:`user<sensor-tendonactuatorfrc-user>`
              -
              -
              -

      .. dropdown:: :ref:`ballquat<sensor-ballquat>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-ballquat-name>`
              - :ref:`joint<sensor-ballquat-joint>`
              - :ref:`cutoff<sensor-ballquat-cutoff>`
              - :ref:`noise<sensor-ballquat-noise>`
            * - :ref:`user<sensor-ballquat-user>`
              -
              -
              -

      .. dropdown:: :ref:`ballangvel<sensor-ballangvel>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-ballangvel-name>`
              - :ref:`joint<sensor-ballangvel-joint>`
              - :ref:`cutoff<sensor-ballangvel-cutoff>`
              - :ref:`noise<sensor-ballangvel-noise>`
            * - :ref:`user<sensor-ballangvel-user>`
              -
              -
              -

      .. dropdown:: :ref:`jointlimitpos<sensor-jointlimitpos>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-jointlimitpos-name>`
              - :ref:`joint<sensor-jointlimitpos-joint>`
              - :ref:`cutoff<sensor-jointlimitpos-cutoff>`
              - :ref:`noise<sensor-jointlimitpos-noise>`
            * - :ref:`user<sensor-jointlimitpos-user>`
              -
              -
              -

      .. dropdown:: :ref:`jointlimitvel<sensor-jointlimitvel>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-jointlimitvel-name>`
              - :ref:`joint<sensor-jointlimitvel-joint>`
              - :ref:`cutoff<sensor-jointlimitvel-cutoff>`
              - :ref:`noise<sensor-jointlimitvel-noise>`
            * - :ref:`user<sensor-jointlimitvel-user>`
              -
              -
              -

      .. dropdown:: :ref:`jointlimitfrc<sensor-jointlimitfrc>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-jointlimitfrc-name>`
              - :ref:`joint<sensor-jointlimitfrc-joint>`
              - :ref:`cutoff<sensor-jointlimitfrc-cutoff>`
              - :ref:`noise<sensor-jointlimitfrc-noise>`
            * - :ref:`user<sensor-jointlimitfrc-user>`
              -
              -
              -

      .. dropdown:: :ref:`tendonlimitpos<sensor-tendonlimitpos>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-tendonlimitpos-name>`
              - :ref:`tendon<sensor-tendonlimitpos-tendon>`
              - :ref:`cutoff<sensor-tendonlimitpos-cutoff>`
              - :ref:`noise<sensor-tendonlimitpos-noise>`
            * - :ref:`user<sensor-tendonlimitpos-user>`
              -
              -
              -

      .. dropdown:: :ref:`tendonlimitvel<sensor-tendonlimitvel>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-tendonlimitvel-name>`
              - :ref:`tendon<sensor-tendonlimitvel-tendon>`
              - :ref:`cutoff<sensor-tendonlimitvel-cutoff>`
              - :ref:`noise<sensor-tendonlimitvel-noise>`
            * - :ref:`user<sensor-tendonlimitvel-user>`
              -
              -
              -

      .. dropdown:: :ref:`tendonlimitfrc<sensor-tendonlimitfrc>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-tendonlimitfrc-name>`
              - :ref:`tendon<sensor-tendonlimitfrc-tendon>`
              - :ref:`cutoff<sensor-tendonlimitfrc-cutoff>`
              - :ref:`noise<sensor-tendonlimitfrc-noise>`
            * - :ref:`user<sensor-tendonlimitfrc-user>`
              -
              -
              -

      .. dropdown:: :ref:`framepos<sensor-framepos>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-framepos-name>`
              - :ref:`objtype<sensor-framepos-objtype>`
              - :ref:`objname<sensor-framepos-objname>`
              - :ref:`reftype<sensor-framepos-reftype>`
            * - :ref:`refname<sensor-framepos-refname>`
              - :ref:`cutoff<sensor-framepos-cutoff>`
              - :ref:`noise<sensor-framepos-noise>`
              - :ref:`user<sensor-framepos-user>`

      .. dropdown:: :ref:`framequat<sensor-framequat>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-framequat-name>`
              - :ref:`objtype<sensor-framequat-objtype>`
              - :ref:`objname<sensor-framequat-objname>`
              - :ref:`reftype<sensor-framequat-reftype>`
            * - :ref:`refname<sensor-framequat-refname>`
              - :ref:`cutoff<sensor-framequat-cutoff>`
              - :ref:`noise<sensor-framequat-noise>`
              - :ref:`user<sensor-framequat-user>`

      .. dropdown:: :ref:`framexaxis<sensor-framexaxis>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-framexaxis-name>`
              - :ref:`objtype<sensor-framexaxis-objtype>`
              - :ref:`objname<sensor-framexaxis-objname>`
              - :ref:`reftype<sensor-framexaxis-reftype>`
            * - :ref:`refname<sensor-framexaxis-refname>`
              - :ref:`cutoff<sensor-framexaxis-cutoff>`
              - :ref:`noise<sensor-framexaxis-noise>`
              - :ref:`user<sensor-framexaxis-user>`

      .. dropdown:: :ref:`frameyaxis<sensor-frameyaxis>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-frameyaxis-name>`
              - :ref:`objtype<sensor-frameyaxis-objtype>`
              - :ref:`objname<sensor-frameyaxis-objname>`
              - :ref:`reftype<sensor-frameyaxis-reftype>`
            * - :ref:`refname<sensor-frameyaxis-refname>`
              - :ref:`cutoff<sensor-frameyaxis-cutoff>`
              - :ref:`noise<sensor-frameyaxis-noise>`
              - :ref:`user<sensor-frameyaxis-user>`

      .. dropdown:: :ref:`framezaxis<sensor-framezaxis>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-framezaxis-name>`
              - :ref:`objtype<sensor-framezaxis-objtype>`
              - :ref:`objname<sensor-framezaxis-objname>`
              - :ref:`reftype<sensor-framezaxis-reftype>`
            * - :ref:`refname<sensor-framezaxis-refname>`
              - :ref:`cutoff<sensor-framezaxis-cutoff>`
              - :ref:`noise<sensor-framezaxis-noise>`
              - :ref:`user<sensor-framezaxis-user>`

      .. dropdown:: :ref:`framelinvel<sensor-framelinvel>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-framelinvel-name>`
              - :ref:`objtype<sensor-framelinvel-objtype>`
              - :ref:`objname<sensor-framelinvel-objname>`
              - :ref:`reftype<sensor-framelinvel-reftype>`
            * - :ref:`refname<sensor-framelinvel-refname>`
              - :ref:`cutoff<sensor-framelinvel-cutoff>`
              - :ref:`noise<sensor-framelinvel-noise>`
              - :ref:`user<sensor-framelinvel-user>`

      .. dropdown:: :ref:`frameangvel<sensor-frameangvel>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-frameangvel-name>`
              - :ref:`objtype<sensor-frameangvel-objtype>`
              - :ref:`objname<sensor-frameangvel-objname>`
              - :ref:`reftype<sensor-frameangvel-reftype>`
            * - :ref:`refname<sensor-frameangvel-refname>`
              - :ref:`cutoff<sensor-frameangvel-cutoff>`
              - :ref:`noise<sensor-frameangvel-noise>`
              - :ref:`user<sensor-frameangvel-user>`

      .. dropdown:: :ref:`framelinacc<sensor-framelinacc>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-framelinacc-name>`
              - :ref:`objtype<sensor-framelinacc-objtype>`
              - :ref:`objname<sensor-framelinacc-objname>`
              - :ref:`cutoff<sensor-framelinacc-cutoff>`
            * - :ref:`noise<sensor-framelinacc-noise>`
              - :ref:`user<sensor-framelinacc-user>`
              -
              -

      .. dropdown:: :ref:`frameangacc<sensor-frameangacc>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-frameangacc-name>`
              - :ref:`objtype<sensor-frameangacc-objtype>`
              - :ref:`objname<sensor-frameangacc-objname>`
              - :ref:`cutoff<sensor-frameangacc-cutoff>`
            * - :ref:`noise<sensor-frameangacc-noise>`
              - :ref:`user<sensor-frameangacc-user>`
              -
              -

      .. dropdown:: :ref:`subtreecom<sensor-subtreecom>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-subtreecom-name>`
              - :ref:`body<sensor-subtreecom-body>`
              - :ref:`cutoff<sensor-subtreecom-cutoff>`
              - :ref:`noise<sensor-subtreecom-noise>`
            * - :ref:`user<sensor-subtreecom-user>`
              -
              -
              -

      .. dropdown:: :ref:`subtreelinvel<sensor-subtreelinvel>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-subtreelinvel-name>`
              - :ref:`body<sensor-subtreelinvel-body>`
              - :ref:`cutoff<sensor-subtreelinvel-cutoff>`
              - :ref:`noise<sensor-subtreelinvel-noise>`
            * - :ref:`user<sensor-subtreelinvel-user>`
              -
              -
              -

      .. dropdown:: :ref:`subtreeangmom<sensor-subtreeangmom>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-subtreeangmom-name>`
              - :ref:`body<sensor-subtreeangmom-body>`
              - :ref:`cutoff<sensor-subtreeangmom-cutoff>`
              - :ref:`noise<sensor-subtreeangmom-noise>`
            * - :ref:`user<sensor-subtreeangmom-user>`
              -
              -
              -

      .. dropdown:: :ref:`insidesite<sensor-insidesite>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-insidesite-name>`
              - :ref:`site<sensor-insidesite-site>`
              - :ref:`objtype<sensor-insidesite-objtype>`
              - :ref:`objname<sensor-insidesite-objname>`
            * - :ref:`cutoff<sensor-insidesite-cutoff>`
              - :ref:`noise<sensor-insidesite-noise>`
              - :ref:`user<sensor-insidesite-user>`
              -

      .. dropdown:: :ref:`distance<sensor-distance>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-distance-name>`
              - :ref:`geom1<sensor-distance-geom1>`
              - :ref:`geom2<sensor-distance-geom2>`
              - :ref:`body1<sensor-distance-body1>`
            * - :ref:`body2<sensor-distance-body2>`
              - :ref:`cutoff<sensor-distance-cutoff>`
              - :ref:`noise<sensor-distance-noise>`
              - :ref:`user<sensor-distance-user>`

      .. dropdown:: :ref:`normal<sensor-normal>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-normal-name>`
              - :ref:`geom1<sensor-normal-geom1>`
              - :ref:`geom2<sensor-normal-geom2>`
              - :ref:`body1<sensor-normal-body1>`
            * - :ref:`body2<sensor-normal-body2>`
              - :ref:`cutoff<sensor-normal-cutoff>`
              - :ref:`noise<sensor-normal-noise>`
              - :ref:`user<sensor-normal-user>`

      .. dropdown:: :ref:`fromto<sensor-fromto>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-fromto-name>`
              - :ref:`geom1<sensor-fromto-geom1>`
              - :ref:`geom2<sensor-fromto-geom2>`
              - :ref:`body1<sensor-fromto-body1>`
            * - :ref:`body2<sensor-fromto-body2>`
              - :ref:`cutoff<sensor-fromto-cutoff>`
              - :ref:`noise<sensor-fromto-noise>`
              - :ref:`user<sensor-fromto-user>`

      .. dropdown:: :ref:`contact<sensor-contact>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-contact-name>`
              - :ref:`geom1<sensor-contact-geom1>`
              - :ref:`geom2<sensor-contact-geom2>`
              - :ref:`body1<sensor-contact-body1>`
            * - :ref:`body2<sensor-contact-body2>`
              - :ref:`subtree1<sensor-contact-subtree1>`
              - :ref:`subtree2<sensor-contact-subtree2>`
              - :ref:`site<sensor-contact-site>`
            * - :ref:`num<sensor-contact-num>`
              - :ref:`data<sensor-contact-data>`
              - :ref:`reduce<sensor-contact-reduce>`
              - :ref:`cutoff<sensor-contact-cutoff>`
            * - :ref:`noise<sensor-contact-noise>`
              - :ref:`user<sensor-contact-user>`
              -
              -

      .. dropdown:: :ref:`e_potential<sensor-e_potential>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-e_potential-name>`
              - :ref:`cutoff<sensor-e_potential-cutoff>`
              - :ref:`noise<sensor-e_potential-noise>`
              - :ref:`user<sensor-e_potential-user>`

      .. dropdown:: :ref:`e_kinetic<sensor-e_kinetic>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-e_kinetic-name>`
              - :ref:`cutoff<sensor-e_kinetic-cutoff>`
              - :ref:`noise<sensor-e_kinetic-noise>`
              - :ref:`user<sensor-e_kinetic-user>`

      .. dropdown:: :ref:`clock<sensor-clock>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-clock-name>`
              - :ref:`cutoff<sensor-clock-cutoff>`
              - :ref:`noise<sensor-clock-noise>`
              - :ref:`user<sensor-clock-user>`

      .. dropdown:: :ref:`user<sensor-user>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-user-name>`
              - :ref:`objtype<sensor-user-objtype>`
              - :ref:`objname<sensor-user-objname>`
              - :ref:`datatype<sensor-user-datatype>`
            * - :ref:`needstage<sensor-user-needstage>`
              - :ref:`dim<sensor-user-dim>`
              - :ref:`cutoff<sensor-user-cutoff>`
              - :ref:`noise<sensor-user-noise>`
            * - :ref:`user<sensor-user-user>`
              -
              -
              -

      .. dropdown:: :ref:`tactile<sensor-tactile>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-tactile-name>`
              - :ref:`geom<sensor-tactile-geom>`
              - :ref:`mesh<sensor-tactile-mesh>`
              - :ref:`user<sensor-tactile-user>`

      .. dropdown:: :ref:`plugin<sensor-plugin>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<sensor-plugin-name>`
              - :ref:`plugin<sensor-plugin-plugin>`
              - :ref:`instance<sensor-plugin-instance>`
              - :ref:`cutoff<sensor-plugin-cutoff>`
            * - :ref:`objtype<sensor-plugin-objtype>`
              - :ref:`objname<sensor-plugin-objname>`
              - :ref:`reftype<sensor-plugin-reftype>`
              - :ref:`refname<sensor-plugin-refname>`
            * - :ref:`user<sensor-plugin-user>`
              -
              -
              -

         .. dropdown:: :ref:`config<plugin-config>`

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`key<plugin-config-key>`
                 - :ref:`value<plugin-config-value>`
                 -
                 -

   .. dropdown:: :ref:`keyframe<keyframe>`


      .. dropdown:: :ref:`key<keyframe-key>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<keyframe-key-name>`
              - :ref:`time<keyframe-key-time>`
              - :ref:`qpos<keyframe-key-qpos>`
              - :ref:`qvel<keyframe-key-qvel>`
            * - :ref:`act<keyframe-key-act>`
              - :ref:`mpos<keyframe-key-mpos>`
              - :ref:`mquat<keyframe-key-mquat>`
              - :ref:`ctrl<keyframe-key-ctrl>`

   .. dropdown:: :ref:`visual<visual>`


      .. dropdown:: :ref:`global<visual-global>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`cameraid<visual-global-cameraid>`
              - :ref:`orthographic<visual-global-orthographic>`
              - :ref:`fovy<visual-global-fovy>`
              - :ref:`ipd<visual-global-ipd>`
            * - :ref:`azimuth<visual-global-azimuth>`
              - :ref:`elevation<visual-global-elevation>`
              - :ref:`linewidth<visual-global-linewidth>`
              - :ref:`glow<visual-global-glow>`
            * - :ref:`offwidth<visual-global-offwidth>`
              - :ref:`offheight<visual-global-offheight>`
              - :ref:`realtime<visual-global-realtime>`
              - :ref:`ellipsoidinertia<visual-global-ellipsoidinertia>`
            * - :ref:`bvactive<visual-global-bvactive>`
              -
              -
              -

      .. dropdown:: :ref:`quality<visual-quality>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`shadowsize<visual-quality-shadowsize>`
              - :ref:`offsamples<visual-quality-offsamples>`
              - :ref:`numslices<visual-quality-numslices>`
              - :ref:`numstacks<visual-quality-numstacks>`
            * - :ref:`numquads<visual-quality-numquads>`
              -
              -
              -

      .. dropdown:: :ref:`headlight<visual-headlight>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`ambient<visual-headlight-ambient>`
              - :ref:`diffuse<visual-headlight-diffuse>`
              - :ref:`specular<visual-headlight-specular>`
              - :ref:`active<visual-headlight-active>`

      .. dropdown:: :ref:`map<visual-map>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`stiffness<visual-map-stiffness>`
              - :ref:`stiffnessrot<visual-map-stiffnessrot>`
              - :ref:`force<visual-map-force>`
              - :ref:`torque<visual-map-torque>`
            * - :ref:`alpha<visual-map-alpha>`
              - :ref:`fogstart<visual-map-fogstart>`
              - :ref:`fogend<visual-map-fogend>`
              - :ref:`znear<visual-map-znear>`
            * - :ref:`zfar<visual-map-zfar>`
              - :ref:`haze<visual-map-haze>`
              - :ref:`shadowclip<visual-map-shadowclip>`
              - :ref:`shadowscale<visual-map-shadowscale>`
            * - :ref:`actuatortendon<visual-map-actuatortendon>`
              -
              -
              -

      .. dropdown:: :ref:`scale<visual-scale>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`forcewidth<visual-scale-forcewidth>`
              - :ref:`contactwidth<visual-scale-contactwidth>`
              - :ref:`contactheight<visual-scale-contactheight>`
              - :ref:`connect<visual-scale-connect>`
            * - :ref:`com<visual-scale-com>`
              - :ref:`camera<visual-scale-camera>`
              - :ref:`light<visual-scale-light>`
              - :ref:`selectpoint<visual-scale-selectpoint>`
            * - :ref:`jointlength<visual-scale-jointlength>`
              - :ref:`jointwidth<visual-scale-jointwidth>`
              - :ref:`actuatorlength<visual-scale-actuatorlength>`
              - :ref:`actuatorwidth<visual-scale-actuatorwidth>`
            * - :ref:`framelength<visual-scale-framelength>`
              - :ref:`framewidth<visual-scale-framewidth>`
              - :ref:`constraint<visual-scale-constraint>`
              - :ref:`slidercrank<visual-scale-slidercrank>`
            * - :ref:`frustum<visual-scale-frustum>`
              -
              -
              -

      .. dropdown:: :ref:`rgba<visual-rgba>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`fog<visual-rgba-fog>`
              - :ref:`haze<visual-rgba-haze>`
              - :ref:`force<visual-rgba-force>`
              - :ref:`inertia<visual-rgba-inertia>`
            * - :ref:`joint<visual-rgba-joint>`
              - :ref:`actuator<visual-rgba-actuator>`
              - :ref:`actuatornegative<visual-rgba-actuatornegative>`
              - :ref:`actuatorpositive<visual-rgba-actuatorpositive>`
            * - :ref:`com<visual-rgba-com>`
              - :ref:`camera<visual-rgba-camera>`
              - :ref:`light<visual-rgba-light>`
              - :ref:`selectpoint<visual-rgba-selectpoint>`
            * - :ref:`connect<visual-rgba-connect>`
              - :ref:`contactpoint<visual-rgba-contactpoint>`
              - :ref:`contactforce<visual-rgba-contactforce>`
              - :ref:`contactfriction<visual-rgba-contactfriction>`
            * - :ref:`contacttorque<visual-rgba-contacttorque>`
              - :ref:`contactgap<visual-rgba-contactgap>`
              - :ref:`rangefinder<visual-rgba-rangefinder>`
              - :ref:`constraint<visual-rgba-constraint>`
            * - :ref:`slidercrank<visual-rgba-slidercrank>`
              - :ref:`crankbroken<visual-rgba-crankbroken>`
              - :ref:`frustum<visual-rgba-frustum>`
              - :ref:`bv<visual-rgba-bv>`
            * - :ref:`bvactive<visual-rgba-bvactive>`
              -
              -
              -

   .. dropdown:: :ref:`default<default>`
      :icon: sync

      .. list-table::
         :class: mjcf-attributes
         :widths: 25 25 25 25

         * - :ref:`class<default-class>`
           -
           -
           -

      .. dropdown:: :ref:`mesh<default-mesh>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`scale<default-mesh-scale>`
              - :ref:`maxhullvert<default-mesh-maxhullvert>`
              - :ref:`inertia<default-mesh-inertia>`
              -

      .. dropdown:: :ref:`material<default-material>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`texture<default-material-texture>`
              - :ref:`emission<default-material-emission>`
              - :ref:`specular<default-material-specular>`
              - :ref:`shininess<default-material-shininess>`
            * - :ref:`reflectance<default-material-reflectance>`
              - :ref:`metallic<default-material-metallic>`
              - :ref:`roughness<default-material-roughness>`
              - :ref:`rgba<default-material-rgba>`
            * - :ref:`texrepeat<default-material-texrepeat>`
              - :ref:`texuniform<default-material-texuniform>`
              -
              -

         .. dropdown:: :ref:`layer<material-layer>`

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`texture<material-layer-texture>`
                 - :ref:`role<material-layer-role>`
                 -
                 -

      .. dropdown:: :ref:`joint<default-joint>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`type<default-joint-type>`
              - :ref:`group<default-joint-group>`
              - :ref:`pos<default-joint-pos>`
              - :ref:`axis<default-joint-axis>`
            * - :ref:`springdamper<default-joint-springdamper>`
              - :ref:`limited<default-joint-limited>`
              - :ref:`actuatorfrclimited<default-joint-actuatorfrclimited>`
              - :ref:`solreflimit<default-joint-solreflimit>`
            * - :ref:`solimplimit<default-joint-solimplimit>`
              - :ref:`solreffriction<default-joint-solreffriction>`
              - :ref:`solimpfriction<default-joint-solimpfriction>`
              - :ref:`stiffness<default-joint-stiffness>`
            * - :ref:`range<default-joint-range>`
              - :ref:`actuatorfrcrange<default-joint-actuatorfrcrange>`
              - :ref:`actuatorgravcomp<default-joint-actuatorgravcomp>`
              - :ref:`margin<default-joint-margin>`
            * - :ref:`ref<default-joint-ref>`
              - :ref:`springref<default-joint-springref>`
              - :ref:`armature<default-joint-armature>`
              - :ref:`damping<default-joint-damping>`
            * - :ref:`frictionloss<default-joint-frictionloss>`
              - :ref:`user<default-joint-user>`
              -
              -

      .. dropdown:: :ref:`geom<default-geom>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`type<default-geom-type>`
              - :ref:`pos<default-geom-pos>`
              - :ref:`quat<default-geom-quat>`
              - :ref:`contype<default-geom-contype>`
            * - :ref:`conaffinity<default-geom-conaffinity>`
              - :ref:`condim<default-geom-condim>`
              - :ref:`group<default-geom-group>`
              - :ref:`priority<default-geom-priority>`
            * - :ref:`size<default-geom-size>`
              - :ref:`material<default-geom-material>`
              - :ref:`friction<default-geom-friction>`
              - :ref:`mass<default-geom-mass>`
            * - :ref:`density<default-geom-density>`
              - :ref:`shellinertia<default-geom-shellinertia>`
              - :ref:`solmix<default-geom-solmix>`
              - :ref:`solref<default-geom-solref>`
            * - :ref:`solimp<default-geom-solimp>`
              - :ref:`margin<default-geom-margin>`
              - :ref:`gap<default-geom-gap>`
              - :ref:`fromto<default-geom-fromto>`
            * - :ref:`axisangle<default-geom-axisangle>`
              - :ref:`xyaxes<default-geom-xyaxes>`
              - :ref:`zaxis<default-geom-zaxis>`
              - :ref:`euler<default-geom-euler>`
            * - :ref:`hfield<default-geom-hfield>`
              - :ref:`mesh<default-geom-mesh>`
              - :ref:`fitscale<default-geom-fitscale>`
              - :ref:`rgba<default-geom-rgba>`
            * - :ref:`fluidshape<default-geom-fluidshape>`
              - :ref:`fluidcoef<default-geom-fluidcoef>`
              - :ref:`user<default-geom-user>`
              -

      .. dropdown:: :ref:`site<default-site>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`type<default-site-type>`
              - :ref:`group<default-site-group>`
              - :ref:`pos<default-site-pos>`
              - :ref:`quat<default-site-quat>`
            * - :ref:`material<default-site-material>`
              - :ref:`size<default-site-size>`
              - :ref:`fromto<default-site-fromto>`
              - :ref:`axisangle<default-site-axisangle>`
            * - :ref:`xyaxes<default-site-xyaxes>`
              - :ref:`zaxis<default-site-zaxis>`
              - :ref:`euler<default-site-euler>`
              - :ref:`rgba<default-site-rgba>`
            * - :ref:`user<default-site-user>`
              -
              -
              -

      .. dropdown:: :ref:`camera<default-camera>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`projection<default-camera-projection>`
              - :ref:`fovy<default-camera-fovy>`
              - :ref:`ipd<default-camera-ipd>`
              - :ref:`resolution<default-camera-resolution>`
            * - :ref:`pos<default-camera-pos>`
              - :ref:`quat<default-camera-quat>`
              - :ref:`axisangle<default-camera-axisangle>`
              - :ref:`xyaxes<default-camera-xyaxes>`
            * - :ref:`zaxis<default-camera-zaxis>`
              - :ref:`euler<default-camera-euler>`
              - :ref:`mode<default-camera-mode>`
              - :ref:`focal<default-camera-focal>`
            * - :ref:`focalpixel<default-camera-focalpixel>`
              - :ref:`principal<default-camera-principal>`
              - :ref:`principalpixel<default-camera-principalpixel>`
              - :ref:`sensorsize<default-camera-sensorsize>`
            * - :ref:`user<default-camera-user>`
              -
              -
              -

      .. dropdown:: :ref:`light<default-light>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`pos<default-light-pos>`
              - :ref:`dir<default-light-dir>`
              - :ref:`bulbradius<default-light-bulbradius>`
              - :ref:`intensity<default-light-intensity>`
            * - :ref:`range<default-light-range>`
              - :ref:`directional<default-light-directional>`
              - :ref:`type<default-light-type>`
              - :ref:`castshadow<default-light-castshadow>`
            * - :ref:`active<default-light-active>`
              - :ref:`attenuation<default-light-attenuation>`
              - :ref:`cutoff<default-light-cutoff>`
              - :ref:`exponent<default-light-exponent>`
            * - :ref:`ambient<default-light-ambient>`
              - :ref:`diffuse<default-light-diffuse>`
              - :ref:`specular<default-light-specular>`
              - :ref:`mode<default-light-mode>`

      .. dropdown:: :ref:`pair<default-pair>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`condim<default-pair-condim>`
              - :ref:`friction<default-pair-friction>`
              - :ref:`solref<default-pair-solref>`
              - :ref:`solreffriction<default-pair-solreffriction>`
            * - :ref:`solimp<default-pair-solimp>`
              - :ref:`gap<default-pair-gap>`
              - :ref:`margin<default-pair-margin>`
              -

      .. dropdown:: :ref:`equality<default-equality>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`active<default-equality-active>`
              - :ref:`solref<default-equality-solref>`
              - :ref:`solimp<default-equality-solimp>`
              -

      .. dropdown:: :ref:`tendon<default-tendon>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`group<default-tendon-group>`
              - :ref:`limited<default-tendon-limited>`
              - :ref:`range<default-tendon-range>`
              - :ref:`solreflimit<default-tendon-solreflimit>`
            * - :ref:`solimplimit<default-tendon-solimplimit>`
              - :ref:`solreffriction<default-tendon-solreffriction>`
              - :ref:`solimpfriction<default-tendon-solimpfriction>`
              - :ref:`frictionloss<default-tendon-frictionloss>`
            * - :ref:`springlength<default-tendon-springlength>`
              - :ref:`width<default-tendon-width>`
              - :ref:`material<default-tendon-material>`
              - :ref:`margin<default-tendon-margin>`
            * - :ref:`stiffness<default-tendon-stiffness>`
              - :ref:`damping<default-tendon-damping>`
              - :ref:`rgba<default-tendon-rgba>`
              - :ref:`user<default-tendon-user>`

      .. dropdown:: :ref:`general<default-general>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`ctrllimited<default-general-ctrllimited>`
              - :ref:`forcelimited<default-general-forcelimited>`
              - :ref:`actlimited<default-general-actlimited>`
              - :ref:`ctrlrange<default-general-ctrlrange>`
            * - :ref:`forcerange<default-general-forcerange>`
              - :ref:`actrange<default-general-actrange>`
              - :ref:`gear<default-general-gear>`
              - :ref:`cranklength<default-general-cranklength>`
            * - :ref:`user<default-general-user>`
              - :ref:`group<default-general-group>`
              - :ref:`actdim<default-general-actdim>`
              - :ref:`dyntype<default-general-dyntype>`
            * - :ref:`gaintype<default-general-gaintype>`
              - :ref:`biastype<default-general-biastype>`
              - :ref:`dynprm<default-general-dynprm>`
              - :ref:`gainprm<default-general-gainprm>`
            * - :ref:`biasprm<default-general-biasprm>`
              - :ref:`actearly<default-general-actearly>`
              -
              -

      .. dropdown:: :ref:`motor<default-motor>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`ctrllimited<default-motor-ctrllimited>`
              - :ref:`forcelimited<default-motor-forcelimited>`
              - :ref:`ctrlrange<default-motor-ctrlrange>`
              - :ref:`forcerange<default-motor-forcerange>`
            * - :ref:`gear<default-motor-gear>`
              - :ref:`cranklength<default-motor-cranklength>`
              - :ref:`user<default-motor-user>`
              - :ref:`group<default-motor-group>`

      .. dropdown:: :ref:`position<default-position>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`ctrllimited<default-position-ctrllimited>`
              - :ref:`forcelimited<default-position-forcelimited>`
              - :ref:`ctrlrange<default-position-ctrlrange>`
              - :ref:`inheritrange<default-position-inheritrange>`
            * - :ref:`forcerange<default-position-forcerange>`
              - :ref:`gear<default-position-gear>`
              - :ref:`cranklength<default-position-cranklength>`
              - :ref:`user<default-position-user>`
            * - :ref:`group<default-position-group>`
              - :ref:`kp<default-position-kp>`
              - :ref:`kv<default-position-kv>`
              - :ref:`dampratio<default-position-dampratio>`
            * - :ref:`timeconst<default-position-timeconst>`
              -
              -
              -

      .. dropdown:: :ref:`velocity<default-velocity>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`ctrllimited<default-velocity-ctrllimited>`
              - :ref:`forcelimited<default-velocity-forcelimited>`
              - :ref:`ctrlrange<default-velocity-ctrlrange>`
              - :ref:`forcerange<default-velocity-forcerange>`
            * - :ref:`gear<default-velocity-gear>`
              - :ref:`cranklength<default-velocity-cranklength>`
              - :ref:`user<default-velocity-user>`
              - :ref:`group<default-velocity-group>`
            * - :ref:`kv<default-velocity-kv>`
              -
              -
              -

      .. dropdown:: :ref:`intvelocity<default-intvelocity>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`ctrllimited<default-intvelocity-ctrllimited>`
              - :ref:`forcelimited<default-intvelocity-forcelimited>`
              - :ref:`ctrlrange<default-intvelocity-ctrlrange>`
              - :ref:`forcerange<default-intvelocity-forcerange>`
            * - :ref:`actrange<default-intvelocity-actrange>`
              - :ref:`inheritrange<default-intvelocity-inheritrange>`
              - :ref:`gear<default-intvelocity-gear>`
              - :ref:`cranklength<default-intvelocity-cranklength>`
            * - :ref:`user<default-intvelocity-user>`
              - :ref:`group<default-intvelocity-group>`
              - :ref:`kp<default-intvelocity-kp>`
              - :ref:`kv<default-intvelocity-kv>`
            * - :ref:`dampratio<default-intvelocity-dampratio>`
              -
              -
              -

      .. dropdown:: :ref:`damper<default-damper>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`forcelimited<default-damper-forcelimited>`
              - :ref:`ctrlrange<default-damper-ctrlrange>`
              - :ref:`forcerange<default-damper-forcerange>`
              - :ref:`gear<default-damper-gear>`
            * - :ref:`cranklength<default-damper-cranklength>`
              - :ref:`user<default-damper-user>`
              - :ref:`group<default-damper-group>`
              - :ref:`kv<default-damper-kv>`

      .. dropdown:: :ref:`cylinder<default-cylinder>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`ctrllimited<default-cylinder-ctrllimited>`
              - :ref:`forcelimited<default-cylinder-forcelimited>`
              - :ref:`ctrlrange<default-cylinder-ctrlrange>`
              - :ref:`forcerange<default-cylinder-forcerange>`
            * - :ref:`gear<default-cylinder-gear>`
              - :ref:`cranklength<default-cylinder-cranklength>`
              - :ref:`user<default-cylinder-user>`
              - :ref:`group<default-cylinder-group>`
            * - :ref:`timeconst<default-cylinder-timeconst>`
              - :ref:`area<default-cylinder-area>`
              - :ref:`diameter<default-cylinder-diameter>`
              - :ref:`bias<default-cylinder-bias>`

      .. dropdown:: :ref:`muscle<default-muscle>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`ctrllimited<default-muscle-ctrllimited>`
              - :ref:`forcelimited<default-muscle-forcelimited>`
              - :ref:`ctrlrange<default-muscle-ctrlrange>`
              - :ref:`forcerange<default-muscle-forcerange>`
            * - :ref:`gear<default-muscle-gear>`
              - :ref:`cranklength<default-muscle-cranklength>`
              - :ref:`user<default-muscle-user>`
              - :ref:`group<default-muscle-group>`
            * - :ref:`timeconst<default-muscle-timeconst>`
              - :ref:`range<default-muscle-range>`
              - :ref:`force<default-muscle-force>`
              - :ref:`scale<default-muscle-scale>`
            * - :ref:`lmin<default-muscle-lmin>`
              - :ref:`lmax<default-muscle-lmax>`
              - :ref:`vmax<default-muscle-vmax>`
              - :ref:`fpmax<default-muscle-fpmax>`
            * - :ref:`fvmax<default-muscle-fvmax>`
              -
              -
              -

      .. dropdown:: :ref:`adhesion<default-adhesion>`
         :icon: dot

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`forcelimited<default-adhesion-forcelimited>`
              - :ref:`ctrlrange<default-adhesion-ctrlrange>`
              - :ref:`forcerange<default-adhesion-forcerange>`
              - :ref:`gain<default-adhesion-gain>`
            * - :ref:`user<default-adhesion-user>`
              - :ref:`group<default-adhesion-group>`
              -
              -

   .. dropdown:: :ref:`custom<custom>`


      .. dropdown:: :ref:`numeric<custom-numeric>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<custom-numeric-name>`
              - :ref:`size<custom-numeric-size>`
              - :ref:`data<custom-numeric-data>`
              -

      .. dropdown:: :ref:`text<custom-text>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<custom-text-name>`
              - :ref:`data<custom-text-data>`
              -
              -

      .. dropdown:: :ref:`tuple<custom-tuple>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`name<custom-tuple-name>`
              -
              -
              -

         .. dropdown:: :ref:`element<tuple-element>`

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`objtype<tuple-element-objtype>`
                 - :ref:`objname<tuple-element-objname>`
                 - :ref:`prm<tuple-element-prm>`
                 -

   .. dropdown:: :ref:`extension<extension>`


      .. dropdown:: :ref:`plugin<extension-plugin>`

         .. list-table::
            :class: mjcf-attributes
            :widths: 25 25 25 25

            * - :ref:`plugin<extension-plugin-plugin>`
              -
              -
              -

         .. dropdown:: :ref:`instance<plugin-instance>`

            .. list-table::
               :class: mjcf-attributes
               :widths: 25 25 25 25

               * - :ref:`name<plugin-instance-name>`
                 -
                 -
                 -

            .. dropdown:: :ref:`config<instance-config>`

               .. list-table::
                  :class: mjcf-attributes
                  :widths: 25 25 25 25

                  * - :ref:`key<instance-config-key>`
                    - :ref:`value<instance-config-value>`
                    -
                    -

