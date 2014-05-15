#|
Copyright (c) 2014 Richard van Roy (pluizer _at_ freeshell _dot_ de)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
|#


;-------------------------------------------------------
; Module definition
;-------------------------------------------------------

(module physics
  *
  (import chicken scheme foreign
          (prefix chipmunk low:))
  (use lolevel srfi-1 srfi-4 srfi-69 data-structures)
  (reexport (except chipmunk
		    create-space
		    space-add-body
		    space-remove-body
		    space-add-shape
		    space-add-static-shape
		    space-remove-shape
		    space-add-constraint
		    space-remove-constraint
		    space-add-poststep-callback
		    space-step
		    space-userdata
		    create-body
		    create-static-body
		    create-circle-shape
		    create-polygon-shape
		    create-box-shape
		    create-pin-joint
		    create-slide-joint
		    create-pivot-joint-with-pivot
		    create-pivot-joint-with-anchors
		    create-groove-joint
		    create-damped-spring
		    create-damped-rotary-spring
		    create-rotary-limit-joint
		    create-ratchet-joint
		    create-gear-joint
		    create-simple-motor
		    space-nearest-point-query
		    space-segment-query
		    space-bb-query
		    space-shape-query
		    space-each-body
		    space-each-shape
		    space-each-constraint
		    body-each-shape
		    body-each-constraint
		    body-each-arbiter
		    body-userdata
		    shape-userdata
		    shape-group
		    shape-layers
		    space-segment-query))
  (require 'chipmunk)

;-------------------------------------------------------
; %% Iterator callbacks %%
;-------------------------------------------------------

; Space iterations

(define-external (_space_body_iter_bridge (c-pointer body) (c-pointer func))
  void
  (((foreign-lambda scheme-object CHICKEN_gc_root_ref c-pointer) func) body))

(define-external (_space_shape_iter_bridge (c-pointer shape) (c-pointer func))
  void
  (((foreign-lambda scheme-object CHICKEN_gc_root_ref c-pointer) func) shape))

(define-external (_space_constraint_iter_bridge (c-pointer constraint) (c-pointer func))
  void
  (((foreign-lambda scheme-object CHICKEN_gc_root_ref c-pointer) func) constraint))

; Body iteration functions

(define-external (_body_shape_iter_bridge (c-pointer shape) (c-pointer func))
  void
  (((foreign-lambda scheme-object CHICKEN_gc_root_ref c-pointer) func) shape))

(define-external (_body_constraint_iter_bridge (c-pointer constraint) (c-pointer func))
  void
  (((foreign-lambda scheme-object CHICKEN_gc_root_ref c-pointer) func) constraint))

(define-external (_body_arbiter_iter_bridge (c-pointer arbiter) (c-pointer func))
  void
  (((foreign-lambda scheme-object CHICKEN_gc_root_ref c-pointer) func) arbiter))

; Collision callback functions

(define-external (_collision_begin_bridge (c-pointer arbiter) (c-pointer space) (c-pointer func-data))
  bool
  (let* ((tuple ((foreign-lambda scheme-object CHICKEN_gc_root_ref c-pointer) func-data))
         (dispatch (car tuple))
         (data (cadr tuple)))
    (apply (dispatch space 'begin-func) (append (list arbiter space) data))))

(define-external (_collision_presolve_bridge (c-pointer arbiter) (c-pointer space) (c-pointer func-data))
  bool
  (let* ((tuple ((foreign-lambda scheme-object CHICKEN_gc_root_ref c-pointer) func-data))
        (dispatch (car tuple))
        (data (cadr tuple)))
    (apply (dispatch space 'presolve-func) (append (list arbiter space) data))))

(define-external (_collision_postsolve_bridge (c-pointer arbiter) (c-pointer space) (c-pointer func-data))
  bool
  (let* ((tuple ((foreign-lambda scheme-object CHICKEN_gc_root_ref c-pointer) func-data))
        (dispatch (car tuple))
        (data (cadr tuple)))
    (apply (dispatch space 'postsolve-func) (append (list arbiter space) data))))

(define-external (_collision_seperate_bridge (c-pointer arbiter) (c-pointer space) (c-pointer func-data))
  bool
  (let* ((tuple ((foreign-lambda scheme-object CHICKEN_gc_root_ref c-pointer) func-data))
        (dispatch (car tuple))
        (data (cadr tuple)))
    (apply (dispatch space 'seperate-func) (append (list arbiter space) data))))

; Queries

(define-external (_nearest_point_bridge (c-pointer shape)
                                        (double distance) (c-pointer vect)
                                        (c-pointer func-data))
  void
  (let* ((tuple ((foreign-lambda scheme-object CHICKEN_gc_root_ref c-pointer) func-data))
        (func (car tuple))
        (data (cadr tuple))
        (r (make-f64vector 2)))
    ; TODO: There must be a better way.
    ((foreign-lambda* void ((c-pointer v) (f64vector r)) "
	memcpy(r, v, sizeof(double)*2);") vect r)
    (apply func (append (list shape distance r data)))))

(define-external (_bb_query (c-pointer shape) (c-pointer func-data))
  void
  (let* ((tuple ((foreign-lambda scheme-object CHICKEN_gc_root_ref c-pointer) func-data))
         (dispatch (car tuple))
         (data (cadr tuple)))
    (apply (dispatch shape 'seperate-func) (append (list shape) data))))

(define-external (_shape_query (c-pointer shape) (c-pointer contact-point) (c-pointer func-data))
  void
  (let* ((tuple ((foreign-lambda scheme-object CHICKEN_gc_root_ref c-pointer) func-data))
         (dispatch (car tuple))
         (data (cadr tuple)))
    (apply (dispatch shape 'seperate-func) (append (list shape contact-point) func-data))))

;-------------------------------------------------------
; %% Helper functions and syntax %%
;-------------------------------------------------------
 
; A function that takes any number of arguments and always returns #t
(define true (lambda _ #t))

; Returns a list with first occurance of /item/ in /list/ removed.
; May shuffle the list.
(define-syntax delete-first-occurance
  (syntax-rules ()
    ((_ item lst pred?)
     (let loop ((nlst lst) (res (list)))
       (if (null? nlst) res
           (if (pred? (car nlst) item)
            (if (null? (cdr nlst)) (append (cdr nlst) res))
            (loop (cdr nlst) (cons (car nlst) res))))))))


(define new-gc-root
  (foreign-lambda* c-pointer ((scheme-object v)) "
	void* ptr = CHICKEN_new_finalizable_gc_root();
	CHICKEN_gc_root_set(ptr, v);
	C_return(ptr);"))

(define free-gc-root
  (foreign-lambda* void ((c-pointer v)) "
	CHICKEN_delete_gc_root(v);"))

; Protects an object from being moved by the garbage collector
; whilst inside body.
; Syntax: (let-bindings ((<variable> <init>) ...) <body>)
(define-syntax let-protect
  (ir-macro-transformer
   (lambda (e i c)
     (apply (lambda (_ bindings #!rest body)
              (let ((v (gensym)))
                (define new-gc-root
                 `(foreign-lambda* ,(i 'c-pointer) ((scheme-object ,(i v)))
                                         ,(sprintf "
			void* ptr = CHICKEN_new_finalizable_gc_root();
	                CHICKEN_gc_root_set(ptr, ~a);
			C_return(ptr);" v)))
                (define free-gc-root
                  `(foreign-lambda* ,(i 'void) ((c-pointer ,(i v))) ,(sprintf "
			CHICKEN_delete_gc_root(~a);" v)))
                `(let ,(map (lambda (b) `(,(car b) (,new-gc-root ,(cadr b)))) bindings)
                    ,@body
                    ,@(map (lambda (b) `(,free-gc-root ,(car b))) bindings))))
            e))))

(define-syntax define-high-wrappers
  (syntax-rules ()
    ((_ free meta (new old) ...)
     (begin
      (define new (lambda args
                    (let ((obj (apply old args)))
                      (hash-table-set! %meta-hash obj meta)
                      (set-finalizer! obj (lambda (obj)
                                            (free obj)
                                            (hash-table-delete! %meta-hash obj))))))
      ...))))

;-------------------------------------------------------
; %% Hash table to store metadata of c objects  %%
;-------------------------------------------------------
(define %meta-hash (make-hash-table pointer=?))
(define (%metadata pointer)
  (hash-table-ref %meta-hash pointer))

;-------------------------------------------------------
; Space
;-------------------------------------------------------

(define-record space-meta
  ; Entities
  bodies
  shapes
  constraints
  ; Default collision handlers
  on-collision-begin
  on-collision-presolve
  on-collision-postsolve
  on-collision-seperate
  ; Custom collision handlers
  ; hash with key: (list [collision-type-a] [collision-type-b])
  ; and value: [dispatch-data-tuple]
  collision-handlers
  ; Poststep callbacks
  poststep-callbacks
  ; Userdaga
  userdata)

(define (%empty-space-meta)
  (make-space-meta
   (list) (list) (list)
   true true true true
   (make-hash-table equal?)
   (list)
   #f))

(define (create-space)
  (let ((space (low:create-space))
        (dispatch-data-tuple
         (new-gc-root
          (list (lambda (space type)
                  (case type
                    ((begin-func)
                     (space-meta-on-collision-begin
                      (%metadata space)))
                    ((presolve-func)
                     (space-meta-on-collision-presolve
                      (%metadata space)))
                    ((postsolve-func)
                     (space-meta-on-collision-postsolve
                      (%metadata space)))
                    ((seperate-func)
                     (space-meta-on-collision-seperate
                      (%metadata space)))
                    (else (error "internal error dispatching callback function."))))
                (list)))))
    (hash-table-set! %meta-hash space (%empty-space-meta))
    (low:%space-default-collision-handler-set! space
					       #$_collision_begin_bridge
					       #$_collision_presolve_bridge
					       #$_collision_postsolve_bridge
					       #$_collision_seperate_bridge
                                               dispatch-data-tuple)
    (set-finalizer! space (lambda (space)
                            (free-gc-root dispatch-data-tuple)
                            (low:space-free space)
                            (hash-table-delete! %meta-hash space)))))

(define (space-bodies space) (space-meta-bodies (%metadata space)))
(define (space-shapes space) (space-meta-shapes (%metadata space)))
(define (space-constraints space) (space-meta-constraints (%metadata space)))


(define space-userdata
  (getter-with-setter
   (lambda (space)
     (space-meta-userdata (%metadata space)))
   (lambda (space value)
     (space-meta-userdata-set! (%metadata space) value))))

(define (space-each-body space func)
  (let-protect ((ptr func))
   (low:space-each-body space #$_space_body_iter_bridge ptr)))

(define (space-each-shape space func)
  (let-protect ((ptr func))
   (low:space-each-shape space #$_space_shape_iter_bridge ptr)))

(define (space-each-constraint space func)
  (let-protect ((ptr func))
   (low:space-each-constraint space #$_space_constraint_iter_bridge ptr)))

(define (space-add-body space body)
  (let ((meta (%metadata space)))
    (space-meta-bodies-set!
     meta (cons body (space-meta-bodies meta)))
    (low:space-add-body space body)))
 
(define (space-remove-body space body)
  (let ((meta (%metadata space)))
   (space-meta-bodies-set!
    meta (delete-first-occurance body (space-meta-bodies meta) pointer=?))
     (low:space-remove-body space body)))

(define (space-add-shape space shape)
  (let ((meta (%metadata space)))
    (space-meta-shapes-set!
     meta (cons shape (space-meta-shapes meta)))
    (low:space-add-shape space shape)))

(define (space-add-static-shape space shape)
  (let ((meta (%metadata space)))
    (space-meta-shapes-set!
     meta (cons shape (space-meta-shapes meta)))
    (low:space-add-static-shape space shape)))

(define (space-remove-shape space shape)
  (let ((meta (%metadata space)))
   (space-meta-shapes-set!
    meta (delete-first-occurance shape (space-meta-shapes meta) pointer=?))
     (low:space-remove-shape space shape)))

(define (space-add-constraint space constraint)
  (let ((meta (%metadata space)))
    (space-meta-constraints-set!
     meta (cons constraint (space-meta-constraints meta)))
    (low:space-add-constraint space constraint)))

(define (space-remove-constraint space constraint)
  (let ((meta (%metadata space)))
   (space-meta-constraints-set!
    meta (delete-first-occurance constraint (space-meta-constraints meta) pointer=?))
     (low:space-remove-constraint space constraint)))

; Collision handling

(define space-on-collision-begin
  (getter-with-setter
   (lambda (space)
     (space-meta-on-collision-begin (%metadata space)))
   (lambda (space func)
     (space-meta-on-collision-begin-set! (%metadata space) func))))

(define space-on-collision-presolve
  (getter-with-setter
   (lambda (space)
     (space-meta-on-collision-presolve (%metadata space)))
   (lambda (space func)
     (space-meta-on-collision-presolve-set! (%metadata space) func))))

(define space-on-collision-postsolve
  (getter-with-setter
   (lambda (space)
     (space-meta-on-collision-postsolve (%metadata space)))
   (lambda (space func)
     (space-meta-on-collision-postsolve-set! (%metadata space) func))))

(define space-on-collision-seperate
  (getter-with-setter
   (lambda (space)
     (space-meta-on-collision-seperate (%metadata space)))
   (lambda (space func)
     (space-meta-on-collision-seperate-set! (%metadata space) func))))

(define (space-add-collision-handler space
                                     collision-type-a collision-type-b
                                     begin-func presolve-func postsolve-func seperate-func
                                     #!rest data)
  (let ((dispatch-data-tuple
         (new-gc-root
          (list
           (lambda (space type)
             (case type
               ((begin-func) begin-func)
               ((presolve-func) presolve-func)
               ((postsolve-func) postsolve-func)
               ((seperate-func) seperate-func)
               (else (error "internal error dispatching callback function."))))
           data))))
    (hash-table-set! (space-meta-collision-handlers (%metadata space))
                     (list collision-type-a collision-type-b)
                     dispatch-data-tuple)
    (low:%space-add-collision-handler space
                                  collision-type-a collision-type-b
                                  #$_collision_begin_bridge
                                  #$_collision_presolve_bridge
                                  #$_collision_postsolve_bridge
                                  #$_collision_seperate_bridge
                                  dispatch-data-tuple)))

(define (space-remove-collision-handler space collision-type-a collision-type-b)
  (let ((handlers (space-meta-collision-handlers (%metadata space)))
        (key (list collision-type-a collision-type-b)))
   (free-gc-root (hash-table-ref handlers key))
   (hash-table-delete! handlers key))
  (low:%space-remove-collision-handler space collision-type-a collision-type-b))

(define (space-add-poststep-callback space func key #!rest data)
  (let ((dispatch-data-tuple
         (new-gc-root
          (list (lambda (space type)
                  (case type
                    ((postsolve-func) func)
                    (else (error "internal error dispatching callback function."))))
                data)))
        (key-root (new-gc-root key)))
    (space-meta-poststep-callbacks-set! (%metadata space)
                                        (cons key-root (cons dispatch-data-tuple
                                                        (space-meta-poststep-callbacks
                                                         (%metadata space)))))
    (low:space-add-poststep-callback space
                                     #$_collision_postsolve_bridge
                                     key-root
                                     dispatch-data-tuple)))

(define (space-step space dt)
  (low:space-step space dt)
  ; Remove all poststep callbacks, there are not needed anymore.
  (map free-gc-root (space-meta-poststep-callbacks (%metadata space)))
  (space-meta-poststep-callbacks-set! (%metadata space) (list)))


;-------------------------------------------------------
; Body
;-------------------------------------------------------

(define-record body-meta
  userdata)

(define-high-wrappers
 low:body-free (make-body-meta #f)
 (create-body low:create-body)
 (create-static-body low:create-static-body))

(define body-userdata
  (getter-with-setter
   (lambda (body)
     (body-meta-userdata (%metadata body)))
   (lambda (body value)
     (body-meta-userdata-set! (%metadata body) value))))

(define (body-each-shape body func)
  (let-protect ((ptr func))
   (low:body-each-shape body #$_body_shape_iter_bridge ptr)))

(define (body-each-constraint body func)
  (let-protect ((ptr func))
   (low:body-each-constraint body #$_body_constraint_iter_bridge ptr)))

(define (body-each-arbiter body func)
  (let-protect ((ptr func))
   (low:body-each-arbiter body #$_body_arbiter_iter_bridge ptr)))

;-------------------------------------------------------
; Shape
;-------------------------------------------------------

(define-record shape-meta
  userdata)

(define-high-wrappers
 low:shape-free (make-shape-meta #f)
 (create-circle-shape low:create-circle-shape)
 (create-polygon-shape low:create-polygon-shape)
 (create-box-shape low:create-box-shape))

(define shape-userdata
  (getter-with-setter
   (lambda (shape)
     (shape-meta-userdata (%metadata shape)))
   (lambda (shape value)
     (shape-meta-userdata-set! (%metadata shape) value))))

(define shape-group
  (getter-with-setter
   (lambda (shape)
    (let ((group-index (low:shape-group shape)))
      (assert (hash-table-exists? %group-symbol-table group-index))
      (hash-table-ref %group-symbol-table group-index)))
   (lambda (shape group)
     (set! (low:shape-group shape) (%group->integer group)))))

(define shape-layers
  (getter-with-setter
   (lambda (shape)
     (%layers->list (low:shape-layers shape)))
   (lambda (shape #!rest value)
     (set! (low:shape-layers shape) (%list->layers value)))))

; Polygon shapes

(define (polygon-shape-vertices shape)
  (map (lambda (n) (low:polygon-shape-vertex-ref shape n))
       (iota (low:polygon-shape-vertex-count))))

;-------------------------------------------------------
; Contraint
;-------------------------------------------------------

(define-record constraint-meta
  userdata)

(define-high-wrappers
  low:constraint-free (make-constraint-meta #f)
  (create-pin-joint low:create-pin-joint)
  (create-slide-joint low:create-slide-joint)
  (create-pivot-joint-with-anchors low:create-pivot-joint-with-anchors)
  (create-pivot-joint-with-pivot low:create-pivot-joint-with-pivot)
  (create-groove-joint low:create-groove-joint)
  (create-damped-spring low:create-damped-spring)
  (create-damped-rotary-spring low:create-damped-rotary-spring)
  (create-rotary-limit-joint low:create-rotary-limit-joint)
  (create-ratchet-joint low:create-ratchet-joint)
  (create-gear-joint low:create-gear-joint)
  (create-simple-motor low:create-simple-motor))

(define constraint-userdata
  (getter-with-setter
   (lambda (constraint)
     (constraint-meta-userdata (%metadata constraint)))
   (lambda (constraint value)
     (constraint-meta-userdata-set! (%metadata constraint) value))))

;-------------------------------------------------------
; Layers
;-------------------------------------------------------

(define (%list->layers v)
  (if (null? v) (bitwise-not 0)
      (let loop ((v (delete-duplicates v)) (r 0))
	(if (null? v) r
	    (loop (cdr v)
		  (bitwise-ior r (arithmetic-shift 1 (car v))))))))

(define (%layers->list v)
  (let loop ((i 0) (r (list)))
    (if (>= i 32) r
	(if (not (zero? (bitwise-and (arithmetic-shift v (- i)) 1)))
	    (loop (+ i 1) (cons i r))
	    (loop (+ i 1) r)))))

;-------------------------------------------------------
; Group
;-------------------------------------------------------

(define %group-symbol-table
  (make-hash-table eq?))

(define (%group->integer #!optional symbol)
  (if symbol
   (if (hash-table-exists? %group-symbol-table symbol)
       (hash-table-ref %group-symbol-table symbol)
       (let ((ix (hash-table-size %group-symbol-table)))
         (hash-table-set! %group-symbol-table symbol ix)
         ix))
   0))

(%group->integer 'no-group)

;-------------------------------------------------------
; Queries
;-------------------------------------------------------

(define (space-nearest-point-query space
                                   point max-distance
                                   layers group
                                   func #!rest data)
  (let-protect ((ptr (list func data)))
               (low:space-nearest-point-query space
                                              point max-distance
                                              layers group
                                              #$_nearest_point_bridge
                                              ptr)))

(define (space-segment-query space
                             vect-start vect-end
                             layers group
                             func #!rest data)
  (let-protect ((ptr (list func data)))
               (low:space-segment-query space
                                        layers group
                                        ; cpSegmentQueryFunc has the same form as
                                        ; cpNearestPointQueryFunc so it can be
                                        ; reused.
                                        #$_nearest_point_bridge
                                        ptr)))
(define (space-bb-query space bb layers group func data)
  (let-protect ((ptr (list func data)))
               (low:space-bb-query space bb layers group
                                   #$_bb_query ptr)))

(define (space-shape-query space shape func #!rest data)
  (let-protect ((ptr (list func data)))
               (low:space-shape-query space shape #$_shape_query ptr)))


)
