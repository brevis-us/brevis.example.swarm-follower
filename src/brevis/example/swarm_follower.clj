(ns brevis.example.swarm-follower
  (:gen-class)
  (:use [brevis.graphics.basic-3D]
        [brevis.physics collision core space utils]
        [brevis.shape box sphere cone]
        [brevis core osd vector camera utils display image input globals]))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; ## Swarm follower.
;;
;; Use your mouse to control the centroid of a swarm.
;; 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; ## Globals

(def num-birds (atom 500))
;(def num-birds (atom 2000))

(def avoidance-distance (atom 25))
;(def boundary 1000)
(def boundary 300)

(def speed 10)
(def max-acceleration 30)

(def swarm-attractor (atom nil)); This should get set in initialize
(def swarm-attractor-weight (atom 0.2))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; ## Birds

(defn bird?
  "Is a thing a bird?"
  [thing]
  (= (get-type thing) :bird))

(defn random-bird-position
  "Returns a random valid bird position."
  [] 
  (vec3 (- (rand boundary) (/ boundary 2)) 
        (- (rand boundary) (/ boundary 2)) 
        (- (rand boundary) (/ boundary 2))))

(defn make-bird
  "Make a new bird. At the specified location."
  [position]  
  (move (make-real {:type :bird
                    :color (vec4 1 0 0 1)
                    :shape (create-cone 10.2 1.5)})
        position))
  
(defn random-bird
  "Make a new random bird."
  []
  (make-bird (random-bird-position)))    

(defn bound-acceleration
  "Keeps the acceleration within a reasonable range."
  [v]  
  (if (> (length v) max-acceleration)
    (mul (div v (length v)) max-acceleration)
    v))

(defn bound-velocity
  "Keeps the acceleration within a reasonable range."
  [v]  
  (if (> (length v) speed)
    (mul (div v (length v)) speed)
    v))

(defn periodic-boundary
  "Change a position according to periodic boundary conditions."
  [pos]
  (let [x (x-val pos)
        y (y-val pos)
        z (z-val pos)]
    (vec3 (cond (> x boundary) (- (mod x boundary) boundary)
                (< x (- boundary)) (mod (- x) boundary)
                :else x)
          (cond (> y boundary) (- (mod y boundary) boundary)
                (< y (- boundary)) (mod (- y) boundary)
                :else y)
          (cond (> z boundary) (- (mod z boundary) boundary)
                (< z (- boundary)) (mod (- z) boundary)
                :else z))))

(defn fly
  "Change the acceleration of a bird."
  [bird]
  (let [;nbrs (filter bird? (get-neighbor-objects bird))      
        ;tmp (println (count nbrs))
        ;tmp (do (doseq [nbr nbrs] (print (get-position nbr))) (println)) 
        bird-pos (get-position bird)
        
        ;; Actual closest bird, a little slow
        ;bird-dists (map #(length-vec3 (sub-vec3 (get-position %) bird-pos)) nbrs)
        #_closest-bird #_(when-not (empty? nbrs)
                          (nth nbrs 
                               (reduce #(if (< (nth bird-dists %1) (nth bird-dists %2)) %1 %2) (range (count bird-dists)))))
        
        ;closest-bird (first nbrs)
        
        closest-bird (get-closest-neighbor bird)
        
        ; This should probably be a function
        new-acceleration (if-not closest-bird
                           ;; No neighbor, move randomly
                           (elmul (vec3 (- (rand) 0.5) (- (rand) 0.5) (- (rand) 0.5))
                                  (mul bird-pos -1.0))
                           (let [dvec (sub bird-pos (get-position closest-bird)) 
                                 len (length dvec)]
                             (add (sub (get-velocity closest-bird) (get-velocity bird)); velocity matching
                                  (if (<= len @avoidance-distance)
                                    ;; If far from neighbor, get closer
                                    dvec
                                    ;; If too close to neighbor, move away
                                    (add (mul dvec -1.0)
                                         (vec3 (rand 0.1) (rand 0.1) (rand 0.1)))))));; add a small random delta so we don't get into a loop           
        new-acceleration (add (mul (sub (get-position @swarm-attractor) bird-pos) @swarm-attractor-weight)
                              new-acceleration)
        new-acceleration (if (zero? (length new-acceleration))
                           new-acceleration
                           (mul new-acceleration (/ 1 (length new-acceleration))))]    
    (set-velocity
      (set-acceleration
        (if (or (> (java.lang.Math/abs (x-val bird-pos)) boundary) 
                (> (java.lang.Math/abs (y-val bird-pos)) boundary) 
                (> (java.lang.Math/abs (z-val bird-pos)) boundary)) 
          (move bird (periodic-boundary bird-pos) #_(vec3 0 25 0))
          bird)
        (bound-acceleration
          new-acceleration
          #_(add (mul (get-acceleration bird) 0.5)
               (mul new-acceleration speed))))
      (bound-velocity (get-velocity bird)))
      ))

;(add-global-update-handler 10 (fn [] (println (get-time) (System/nanoTime))))

(enable-kinematics-update :bird); This tells the simulator to move our objects
(add-update-handler :bird fly); This tells the simulator how to update these objects
;(add-parallel-update-handler :bird fly); This tells the simulator how to update these objects (in parallel)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; ## Swarm attractors

(defn create-swarm-attractor
  "Create an object that will attract the swarm"
  [position]
  (move (make-real {:type :swarm-attractor
                    :color (vec4 1 0 0 0.7)
                    :shape (create-sphere 25.0)})
        position))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; ## Input handling

(defn swarm-attractor-input-handlers
  "Define the default input handlers."
  []
  (swap! *gui-state* assoc :keyspeed 1)
  ;(Keyboard/enableRepeatEvents false)
  ;(println "repeat keys?" (Keyboard/areRepeatEventsEnabled))
  (add-input-handler :key-press
                     {:key-id "I"}
                     #(swap! *gui-state* assoc :fullscreen (not (:fullscreen @*gui-state*))))
  (add-input-handler :key-press
                     {:key-id "Q"}
                     #(swap! *gui-state* update-in [:keyspeed] (partial * 1.1)))
  (add-input-handler :key-press
                     {:key-id "E"}
                     #(swap! *gui-state* update-in [:keyspeed] (partial * 0.9)))
  (add-input-handler :key-press
                     {:key-id "X"}
                     (let [timer (atom 0)]
                       (fn []
                         (when (> (- (java.lang.System/nanoTime) @timer) 1000000000) 
                           #_(println "hit pause" (:paused @*gui-state*) @timer (- (java.lang.System/nanoTime) @timer))
                           (reset! timer (java.lang.System/nanoTime))
                           (swap! *gui-state* update-in [:paused] #(not %))))))
  (add-input-handler :key-press
                     {:key-id "D"}
                     #(.moveFromLook (:camera @*gui-state*) (- (:keyspeed @*gui-state*)) 0 0 ))                     
  (add-input-handler :key-press
                     {:key-id "W"}
                     #(.moveFromLook (:camera @*gui-state*) 0 0 (:keyspeed @*gui-state*)))                     
  (add-input-handler :key-press
                     {:key-id "A"}
                     #(.moveFromLook (:camera @*gui-state*) (:keyspeed @*gui-state*) 0 0))                     
  (add-input-handler :key-press
                     {:key-id "S"}
                     #(.moveFromLook (:camera @*gui-state*) 0 0 (- (:keyspeed @*gui-state*))))                     
  (add-input-handler :key-press
                     {:key-id "C"}
                     #(.moveFromLook (:camera @*gui-state*) 0 (- (:keyspeed @*gui-state*)) 0))
  (add-input-handler :key-press
                     #_{:key-id "LSHIFT"} ;; too annoying with os x
                     {:key-id "Z"}
                     #_#(.moveFromLook (:camera @*gui-state*) 0 (- keyspeed) 0)
                     #(.moveFromLook (:camera @*gui-state*) 0 (:keyspeed @*gui-state*) 0))
  (add-input-handler :key-press
                     {:key-id "P"}
                     #(swap! *gui-state* assoc :pause (not (:pause @*gui-state*))))
  (add-input-handler :key-press
                     {:key-id "O"}
                     #(screenshot (str "brevis_screenshot_" (System/currentTimeMillis) ".png")))
  (add-input-handler :key-press
                     {:key-id "ESCAPE"}
                     #(swap! *gui-state* assoc :close-requested true))
  ; Usual rotate with mouse clicke
  #_(add-input-handler :mouse-click
                      {:mouse-button "LEFT"}
                      #(.rotateFromLook (:camera @*gui-state*) (- (get-mouse-dy)) (get-mouse-dx) 0))
  
  (add-input-handler :mouse-click
                      {:mouse-button "LEFT"}
                      #(let [p (get-position @swarm-attractor)]
                         (move @swarm-attractor (vec3 (- (* (/ (get-mouse-x) (.width (:camera @*gui-state*)))
                                                            2 boundary)
                                                         boundary)
                                                      (- (* (/ (get-mouse-y) (.height (:camera @*gui-state*)))
                                                            2 boundary)
                                                         boundary)
                                                      (z-val-vec3 p))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; ## Collision handling
;;
;; Collision functions take [collider collidee] and return [collider collidee]
;; This is only called once per pair of colliding objects.

(defn bump
  "Collision between two birds."
  [bird1 bird2]  
  [(set-color bird1 (vec4 (rand) (rand) (rand) 1))
   bird2])

(defn land
  "Collision between a bird and the floor."
  [bird floor]
  [(set-velocity (set-acceleration bird (vec3 0 10.5 0)) (vec3 0 10.0 0));; maybe move as well       
   floor])

(add-collision-handler :bird :bird bump)
(add-collision-handler :bird :floor land)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; ## brevis control code

(defn initialize-simulation
  "This is the function where you add everything to the world."
  []  
  ;(swap! brevis.globals/*gui-state* assoc :gui false)
  (init-world)
  (init-view)  
  
  #_(change-skybox
     ["img/night_skybox/front.jpg"
      "img/night_skybox/left.jpg"
      "img/night_skybox/back.jpg"
      "img/night_skybox/right.jpg"
      "img/night_skybox/up.jpg"
      "img/night_skybox/down.jpg"])
  ;(swap! brevis.globals/*gui-state* assoc :gui false)
  #_(.moveFromLook (:camera @brevis.globals/*gui-state*) 0 100 0)
  #_(set-dt 0.1)
  
  #_(set-camera-information (vec3 -10.0 -50.0 -200.0) (vec4 1.0 0.0 0.0 0.0))
  ;(set-camera-information (vec3 -10.0 57.939613 -890.0) (vec4 1.0 0.0 0.0 0.0))
  ;(set-camera-information (vec3 -10.0 164.3215 -408.42572) (vec4 1.0 0.0 0.0 0.0))
  (set-camera-information (vec3 -10.0 32.89055 -410.34692) (vec4 1.0 0.0 0.0 0.0))
  
  ;(.setParallel *java-engine* true)
  
  #_(disable-skybox)
  
  ;; Record screenshots
  #_(add-global-update-handler 10
                              (fn []
                                (screenshot (str "screenshot_" (format "%05d" (int (get-time))) ".png"))
                                ))
  
  (set-dt 1)
  (set-neighborhood-radius 50)
  (default-display-text)
  ;(add-object (move (make-floor 500 500) (vec3 0 (- boundary) 0)))
  (reset! swarm-attractor (add-object (create-swarm-attractor (vec3 0 0 0))))
  (dotimes [_ @num-birds]
    (add-object (random-bird))))

;; Start zee macheen
(defn -main [& args]
  #_(start-nogui initialize-simulation)
  (if-not (empty? args)
    (start-nogui initialize-simulation)
    (start-gui initialize-simulation               
               java-update-world
               swarm-attractor-input-handlers
               #_default-input-handlers
               )))

(autostart-in-repl -main)
