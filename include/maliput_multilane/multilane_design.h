/// @file multilane_design.h
/// @page multilane_design Multilane Design
/// @author Matt Marjanović
/// @author Chien-Liang Fok
/// @author Agustin Alba Chicar
/// @date April 21, 2020
/// @tableofcontents
///
/// @section concrete_implementation_maliput_multilane Concrete Implementation: `maliput::multilane`
///
/// So-named because it admits multiple `Lanes` per
/// `Segment`, an advance over its predecessor (`monolane`) which only
/// admitted a single `Lane` per `Segment`.
///
/// `multilane`  is an implementation of the
/// `maliput` geometry API which synthesizes a road network from a small set
/// of primitive building blocks, mimicking techniques used in the geometric
/// design of real roads.  The basic geometry of a `Segment` is derived
/// from the combination of a plane curve, an elevation
/// function, and a superelevation function, combined together to define a
/// ruled surface.  A `Segment` has a longitudinal *reference curve*
/// (similar to a `Lane`'s centerline) and each of the `Lanes` of a
/// `Segment` is defined via a constant lateral offset, along the segment
/// surface, from that reference curve.
///
/// Three coordinate frames are involved in the following discussion:
///  * @f$(x,y,z)@f$ is a position in the `World`-frame.
///  * @f$(s,r,h)_{LANE,i}@f$ is a position in the `Lane`-frame of the `Lane` with
///    index @f$i@f$.
///  * @f$(p,r,h)_{SEG}@f$ is a position in a curvilinear reference frame of
///    the `Segment`, analogous to @f$(s,r,h)_{LANE,i}@f$ for a `Lane`.
///    The parameter @f$p_{SEG} \in [0, 1]@f$ spans the `Segment` longitudinally.
///    @f$r_{SEG}@f$ is a lateral offset from the `Segment`'s reference curve,
///    along the `Segment` surface. @f$h_SEG@f$ is height above the surface.
///
/// @subsection segment_geometry `Segment` Geometry
///
/// > TODO Reconsider the use of the word "geometry" below.
/// > The geometry of a `Segment` is completely derived from a map
/// >
/// > @f[
/// > W: (p,r,h)_{SEG} \mapsto (x,y,z)
/// > @f]
/// >
/// > which we will construct in stages, starting with the `Segment` reference curve
/// >
/// > @f[
/// > W(p_{SEG}) \equiv W(p_{SEG},0,0),
/// > @f]
/// >
/// > followed by the `Segment` surface
/// >
/// > @f[
/// > W(p_{SEG},r_{SEG}) \equiv W(p_{SEG},r_{SEG},0).
/// > @f]
///
/// The construction of @f$W(p_{SEG},r_{SEG},h_{SEG})@f$ will involve
/// three fundamental functions, @f$G_\text{xy}@f$, @f$G_z@f$, and @f$\Theta@f$.
///
/// The first fundamental function @f$G_\text{xy}@f$ defines a two dimensional
/// *planar primitive curve* in the @f$xy@f$ -plane:
///
/// @f[
/// G_{xy}: p_{SEG} \mapsto (x,y).
/// @f]
///
/// This curve establishes the basic geometric primitive of the `Segment`
/// (e.g., "constant-radius arc").
/// We define @f$l@f$ as a path-length along this plane curve, in the range
/// @f$[0, l_\text{max}]@f$, where @f$l_\text{max}@f$ is the total path-length
/// of the curve.  @f$G_{xy}@f$ is specifically parameterized such that
///
/// @f[
/// p_{SEG} \equiv \frac{l}{l_\text{max}};
/// @f]
///
/// in other words, @f$p_{SEG}@f$ is linear in path-length along the planar
/// primitive curve and @f$p_{SEG} \in [0,1]@f$.
///
/// The second fundamental function @f$G_z@f$ specifies elevation above the
/// @f$(xy)@f$-plane (albeit with a peculiar scale factor):
///
/// @f[
/// G_z: p_{SEG} \mapsto \frac{1}{l_\text{max}}z
/// @f]
///
/// Taking @f$G_{xy} = (G_x, G_y)@f$ and @f$G_z@f$ together,
///
/// > @f[
/// > \left(\begin{array}{c} G_{xy}\\ l_\text{max}G_z \end{array}\right): p_{SEG} \mapsto
/// >   \left(\begin{array}{c}x\\y\\z\end{array}\right)
/// > @f]
///
/// @f[
/// \left(\begin{array}{c}x\\y\\z\end{array}\right) =
/// W(p_{SEG}) =
/// \left(\begin{array}{c} G_x(p_{SEG})\\
///                        G_y(p_{SEG})\\
///                        l_\text{max}G_z(p_{SEG}) \end{array}\right)
/// @f]
///
/// defines the three dimensional *reference curve* @f$W(p_{SEG})@f$ for the `Segment`.
/// @f$G_z@f$ is constructed with the scale factor of @f$1/l_\text{max}@f$ specifically
/// so that:
///
/// @f[
/// \begin{eqnarray*}
///       z & = & l_\text{max} G_z(p_{SEG})\\
///         & = & l_\text{max} G_z\left(\frac{l}{l_\text{max}}\right)\\
/// \dot{z} & = & \frac{dz}{dl} = \frac{d}{dp_{SEG}}G_z(p_{SEG})
/// \end{eqnarray*}
/// @f]
///
/// This allows us to derive the first derivative of @f$G_z@f$ directly from
/// the `World`-frame slope @f$\dot{z} = \frac{dz}{dl}@f$ of the segment
/// surface along its reference curve.  This is convenient because @f$\dot{z}@f$
/// is what a road designer would nominally specify as the "slope of the road"
/// or the "grade of the road".
///
/// The third fundamental function @f$\Theta@f$ specifies the superelevation of
/// the `Segment` surface:
///
/// @f[
/// \Theta: p_{SEG} \mapsto \frac{1}{l_\text{max}}\theta
/// @f]
///
/// Superelevation @f$\theta@f$ is the "twist" in a road, given as a right-handed
/// angle of rotation around the tangent of the reference curve @f$W(p_{SEG})@f$.
/// Zero superelevation leaves the surface parallel with the
/// @f$xy@f$ plane. Note that superelevation becomes ambiguous when the
/// tangent of the reference curve points in the @f$\hat{z}@f$ direction.
///
/// As with @f$G_z@f$, @f$\Theta@f$ is scaled so that:
///
/// @f[
/// \begin{eqnarray*}
///       \theta & = & l_\text{max} \Theta\left(\frac{l}{l_\text{max}}\right)\\
/// \dot{\theta} & = &
///               \frac{d\theta}{dl} = \frac{d}{dp_{SEG}}\Theta(p_{SEG})
/// \end{eqnarray*}
/// @f]
///
/// > With the three fundamental functions in hand, we can express the orientation
/// > of the @f$(\hat{p},\hat{r},\hat{h})_{SEG}@f$ frame along the reference curve,
/// > with respect to the `World`-frame, as a roll/pitch/yaw rotation:
///
/// We use all three fundamental functions to define a rotation
///
/// @f[
/// \begin{align*}
/// \mathbf{R}(p_{SEG}) &=
///  \mathbf{R}_{\gamma(p_{SEG})}
///  \mathbf{R}_{\beta(p_{SEG})} \mathbf{R}_{\alpha(p_{SEG})}
/// \end{align*}
/// @f]
///
/// where
///
/// @f[
/// \begin{align*}
///   \mathbf{R}_{\gamma(p_{SEG})} &=
///   \left(\begin{array}{rrr}
///   \cos\gamma & -\sin\gamma & 0 \\
///   \sin\gamma &  \cos\gamma & 0 \\
///            0 &           0 & 1
///   \end{array}\right) & \text{(yaw)}\\
/// \end{align*}
/// @f]
///
/// @f[
/// \begin{align*}
///   \mathbf{R}_{\beta(p_{SEG})}  &=
///   \left(\begin{array}{rrr}
///    \cos\beta & 0 & \sin\beta \\
///            0 & 1 &         0 \\
///   -\sin\beta & 0 & \cos\beta
///   \end{array}\right) & \text{(pitch)} \\
/// \end{align*}
/// @f]
///
/// @f[
/// \begin{align*}
///   \mathbf{R}_{\alpha(p_{SEG})} &=
///   \left(\begin{array}{rrr}
///   1 &          0 &           0 \\
///   0 & \cos\alpha & -\sin\alpha \\
///   0 & \sin\alpha &  \cos\alpha
///   \end{array}\right) & \text{(roll)}
/// \end{align*}
/// @f]
///
/// and
///
/// @f[
/// \begin{align*}
/// \gamma(p_{SEG}) &=
///   \mathrm{atan2}\negthickspace\left(\frac{dG_y}{dp_{SEG}},
///                       \frac{dG_x}{dp_{SEG}}\right) & \text{(yaw)}\\
/// \beta(p_{SEG})  &=
///   \arctan\negthickspace\left(\frac{dG_z}
///                                         {dp_{SEG}}\right)
/// & \text{(pitch)} \\
/// \alpha(p_{SEG}) &= l_\text{max}\Theta(p_{SEG}) & \text{(roll)}
/// \end{align*}
/// @f]
///
/// > Note that @f$\hat{p}_{SEG}@f$ is solely determined by @f$W(p_{SEG})@f$,
/// > and as expected,
/// > @f$\hat{p}_{SEG} = \frac{W'(p_{SEG})}{\lVert W'(p_{SEG})\rVert}@f$.
///
/// With @f$\mathbf{R}(p_{SEG})@f$ , we can extend the `Segment` reference curve @f$W(p_{SEG})@f$
/// to construct the `Segment` *surface* @f$W(p_{SEG}, r_{SEG})@f$ as:
///
/// @f[
/// \begin{align*}
/// \left(\begin{array}{c}x\\y\\z\end{array}\right) =
/// W(p_{SEG},r_{SEG}) = \left(
/// \begin{array}{c}
///    G_{xy}(p_{SEG})\\
///    l_\text{max} G_z(p_{SEG})
/// \end{array} \right) +
/// \mathbf{R}(p_{SEG})\negthickspace
/// \begin{pmatrix}
/// 0\\ r_{SEG} \\ 0 \end{pmatrix}.
/// \end{align*}
/// @f]
///
/// This function defines a *ruled surface*.  For any @f$p_{SEG}@f$,
/// @f$W(p_{SEG},r_{SEG})@f$ is linear in @f$r_{SEG}@f$ and motion along
/// @f$r_{SEG}@f$ is in a straight line.
///
/// Now that we have the surface embedding @f$W(p_{SEG},r_{SEG})@f$,
/// we can derive
/// the basis vectors @f$(\hat{p}, \hat{r}, \hat{h})_{SEG}@f$ along the surface
/// and the corresponding orientation @f$\mathbf{R}(p_{SEG},r_{SEG})@f$:
///
/// @f[
/// \begin{align*}
/// \mathbf{R}(p_{SEG},r_{SEG}) &=
///                      \begin{pmatrix}\hat{p} & \hat{r} & \hat{h}\end{pmatrix}\\
/// \hat{p}_{SEG} &=
///  \frac{\partial_{p_{SEG}} W(p_{SEG},r_{SEG})}{\lVert\partial_{p_{SEG}} W(p_{SEG},r_{SEG})\rVert}\\
/// \hat{r}_{SEG} &=
///  \frac{\partial_{r_{SEG}} W(p_{SEG},r_{SEG})}{\lVert\partial_{r_{SEG}} W(p_{SEG},r_{SEG})\rVert}\\
/// \hat{h}_{SEG} &= \hat{p}_{SEG} \times \hat{r}_{SEG}
/// \end{align*}
/// @f]
///
/// A few things are worth noting at this point:
///
///  * @f$\hat{r}_{SEG} = \mathbf{R}(p_{SEG}) \begin{pmatrix}0\\1\\0\end{pmatrix}@f$.
///    Thus, @f$\hat{r}_{SEG}@f$ is independent of @f$r_{SEG}@f$.
///  * @f$\mathbf{R}(p_{SEG},r_{SEG}) = \mathbf{R}(p_{SEG})@f$ along
///    @f$r_{SEG} = 0@f$ just as it should be; the orientation along the
///    `Segment`'s reference curve is consistent in both expressions.
///  * @f$\hat{p}_{SEG}@f$ is *not necessarily* independent of
///    @f$r_{SEG}@f$.  Consequently, @f$\mathbf{R}(p_{SEG},r_{SEG})@f$ is not
///    necessarily equal to @f$\mathbf{R}(p_{SEG})@f$ for
///    @f$r_{SEG}\ne 0@f$.  This will become important when we try to
///    join `Segments` end-to-end preserving @f$G^1@f$ continuity, discussed in
///    section @ref ensuring_g1_contiguity .
///
/// *Finally*, with @f$\mathbf{R}(p_{SEG},r_{SEG})@f$ in hand (and points 1 and
/// 2 above), we can define the complete volumetric world map
/// @f$W(p_{SEG},r_{SEG},h_{SEG})@f$ for a `Segment`'s geometry:
///
/// @f[
/// \begin{align*}
/// \begin{pmatrix}x\\y\\z\end{pmatrix} = W(p_{SEG},r_{SEG},h_{SEG}) = \left(
/// \begin{array}{c}
///    G_x(p_{SEG})\\
///    G_y(p_{SEG})\\
///    l_\text{max} G_z(p_{SEG})
/// \end{array} \right) +
/// \mathbf{R}(p_{SEG},r_{SEG})\negthickspace
/// \begin{pmatrix}
/// 0\\ r_{SEG} \\ h_{SEG} \end{pmatrix}.
/// \end{align*}
/// @f]
///
/// This is simply @f$W(p_{SEG},r_{SEG})@f$ displaced by @f$h_{SEG}@f$ along
/// the surface normal @f$\hat{h}_{SEG}@f$.
///
/// @subsection lane_geometry `Lane` Geometry
///
/// A `Lane` derives its geometry from its `Segment`.  In `multilane`, the
/// centerline of the `Lane` with index @f$i@f$ is a parallel curve with a constant
/// lateral offset @f$r_i@f$ from the reference curve (at @f$r_{SEG} = 0@f$) of the
/// `Segment`.  We can express this relationship as a transform between
/// @f$(s,r,h)_{LANE,i}@f$ (`Lane`-frame) and @f$(p,r,h)_{SEG}@f$
/// (`Segment`-frame):
///
/// @f[
/// \begin{align*}
/// \begin{pmatrix} p_{SEG}\\
///                 r_{SEG}\\
///                 h_{SEG} \end{pmatrix}
/// &= \begin{pmatrix}    P(s_{LANE,i})\\
///                    r_{LANE,i} + r_i\\
///                          h_{LANE,i} \end{pmatrix}
/// \end{align*}
/// @f]
///
/// The tricky part here is @f$P:s_{LANE,i} \mapsto p_{SEG}@f$, which relates
/// @f$s_{LANE,i}@f$ to @f$p_{SEG}@f$, and involves the
/// path-length integral over @f$W(p_{SEG},r_{SEG})@f$.
///
/// `maliput` defines @f$s_{LANE,i}@f$ as the path-length along a `Lane`'s
/// centerline, and in `multilane` that centerline is a curve with constant
/// @f$r_{SEG} = r_i@f$.  Thus:
///
/// @f[
/// \begin{align*}
/// s_{LANE,i} = S(p_{SEG}) &=
///  \left. \int \left\lVert \partial_{p_{SEG}}W(p_{SEG}, r_{SEG})
///  \right\rVert dp_{SEG} \right\rvert_{r_{SEG} = r_i}.
/// \end{align*}
/// @f]
///
/// The function @f$P@f$ that we need is the inverse of the path-integral @f$S@f$.
///
/// Unfortunately, there is generally no closed-form solution for either
/// @f$S@f$ or @f$P@f$, particularly if the surface is not flat.  `multilane` will
/// compute @f$P(s_{LANE,i})@f$ and @f$S(p_{SEG})@f$ analytically if
/// possible (e.g., for some flat surfaces) and otherwise will use more costly
/// numerical methods to ensure accurate results. Which makes us
/// wonder, perhaps the `Lane`-frame of `maliput` would be better off
/// using an arbitrary longitudinal parameter @f$p_{LANE,i}@f$ which could
/// be converted to a distance @f$s_{LANE,i}@f$ on demand, instead of the other
/// way around.
///
/// > TODO: Derivation of orientation at arbitrary @f$(s,r,h)_{LANE,i}@f$ point.
/// >
/// > TODO: Derivation of motion-derivatives.
/// >
/// > TODO: Derivation of surface/path curvatures.
///
/// ### Available Implementations of @f$G_\text{xy}@f$, @f$G_z@f$, and @f$\Theta@f$
///
/// `multilane` currently implements one form for each of @f$G_{xy}@f$,
/// @f$G_z@f$, and @f$\Theta@f$.  @f$G_{xy}@f$ is implemented for a constant curvature
/// arc (which includes zero curvature, i.e., straight line segments).
/// Elevation @f$G_z@f$ and superelevation @f$\Theta@f$ are implemented for cubic
/// polynomials.  These forms were chosen because they provide the smallest,
/// simplest set of primitives that allow for the assembly of fully
/// three-dimensional road networks that maintain @f$G^1@f$ continuity across
/// segment boundaries.
///
/// The exact form that @f$G_{xy}@f$ takes is:
///
/// @f[
/// \begin{align*}
/// \begin{pmatrix} x\\ y \end{pmatrix} = G_\text{xy}(p_{SEG}) &=
///     \begin{pmatrix}x_0\\ y_0\end{pmatrix} +
///     \left\lbrace \begin{array}
///         \frac{1}{\kappa}\begin{pmatrix}
///           \cos(\kappa l_\text{max} p_{SEG} + \gamma_0 - \frac{\pi}{2}) - \cos(\gamma_0 - \frac{\pi}{2})\\
///           \sin(\kappa l_\text{max} p_{SEG} + \gamma_0 - \frac{\pi}{2}) - \sin(\gamma_0 - \frac{\pi}{2})
///           \end{pmatrix} & \text{for }\kappa > 0\\
///         l_\text{max} p_{SEG}
///           \begin{pmatrix}\cos{\gamma_0}\\ \sin{\gamma_0}\end{pmatrix}
///           & \text{for }\kappa = 0\\
///         \frac{1}{\kappa}\begin{pmatrix}
///           \cos(\kappa l_\text{max} p_{SEG} + \gamma_0 + \frac{\pi}{2}) - \cos(\gamma_0 + \frac{\pi}{2})\\
///           \sin(\kappa l_\text{max} p_{SEG} + \gamma_0 + \frac{\pi}{2}) - \sin(\gamma_0 + \frac{\pi}{2})
///           \end{pmatrix} & \text{for }\kappa < 0\\
///     \end{array} \right\rbrace
/// \end{align*}
/// @f]
///
///
/// where @f$\kappa@f$ is the signed curvature (positive is
/// counterclockwise/leftward), @f$l_\text{max}@f$ is the arc length,
/// @f$\begin{pmatrix}x_0\\y_0\end{pmatrix}@f$ is the
/// starting point of the arc, and @f$\gamma_0@f$ is the initial yaw of the
/// (tangent) of the arc (with @f$\gamma_0 = 0@f$ in the @f$+\hat{x}@f$
/// direction).  Note that the @f$\kappa = 0@f$ expression is simply a line
/// segment of length @f$l_\text{max}@f$, and it is the limit of the @f$\kappa
/// \neq 0@f$ expressions as @f$\kappa \to 0@f$.
///
/// With regards to geometric road design, a constant curvature
/// @f$G_\text{xy}@f$ does not provide a complete toolkit.  Most road designs
/// involve clothoid spirals, which are plane curves with curvature that
/// is /linear/ in path length.This is so that vehicles can navigate
/// roads using continuous changes in steering angle, and, likewise, so that
/// their occupants will experience continuous changes in radial acceleration.
/// `multilane` is expected to extend support for clothoid @f$G_\text{xy}@f$
/// in the future.
///
/// For @f$G_z@f$ and @f$\Theta@f$, a cubic polynomial is the lowest-degree polynomial
/// which allows for independently specifying the value and the first derivative
/// at both endpoints.  Thus, @f$G_z@f$ takes the form:
///
/// @f[
/// \begin{align*}
/// \begin{split}
/// \frac{1}{l_\text{max}}z = G_z(p_{SEG}) &=
///  \frac{z_0}{l_\text{max}} +
///  \dot{z_0} p_{SEG} +
///  \left(\frac{3(z_1 - z_0)}{l_\text{max}} - 2\dot{z_0} - \dot{z_1}\right)
///    p_{SEG}^2 \\
///  &\quad + \left(\dot{z_0} + \dot{z_1} - \frac{2(z_1 - z_0)}{l_\text{max}}\right)
///    p_{SEG}^3
/// \end{split}
/// \end{align*}
/// @f]
///
/// where @f$z_0@f$ and @f$z_1@f$ are the initial and final elevation
/// respectively, and @f$\dot{z_0}@f$ and @f$\dot{z_1}@f$ are the initial and
/// final @f$\frac{dz}{dl}@f$, which is simply the slope of the road as
/// measured by the intuitive "rise over run".  @f$\Theta@f$ has an identical
/// expression, with every @f$z@f$ replaced by @f$\theta@f$.  Note that
/// @f$\dot{\theta} = \frac{d\theta}{dl}@f$, the rate of twisting of the road,
/// is not particularly intuitive, but that's ok because in general
/// @f$\dot{\theta_0}@f$ and @f$\dot{\theta_1}@f$ will be set by `multilane` and
/// not by the road designer, as we will see in section @ref ensuring_g1_contiguity .
///
/// @subsection ensuring_g1_contiguity Ensuring G¹ Continuity
///
/// > TODO:  Tell me more!
///
/// @subsection builder_helper_interface `Builder` helper interface
///
/// Users are not expected to assemble a `multilane::RoadGeometry` by
/// constructing individual instances of `multilane::Lane`, etc, by hand.
/// Instead, `multilane` provides a `Builder` interface which handles
/// many of the constraints involved in constructing a valid `RoadGeometry`.
///
/// > TODO:  Tell me more!
///
/// @subsection yaml_file_format YAML file format
///
/// Multilane provides two loader methods
/// ( @ref maliput::multilane::Load() "Load()" and
/// @ref maliput::multilane::LoadFile() "LoadFile()" ) that will parse a
/// YAML file or string by calling appropriate
/// @ref maliput::multilane::Builder "Builder" methods to create a
/// `RoadGeometry`.
///
/// The serialization is a fairly straightforward mapping of the
/// @ref maliput::multilane::Builder "Builder" interface onto YAML.
///
/// The basic idea is, however:
///
/// - general parameters (i.e., lane_width, elevation bounds, linear and
///   angular tolerance)
/// - a collection of named 'points', which are specifications of explicitly
///   named Endpoints
/// - a collection of named 'connections', whose start Endpoints are specified
///   by reference to either a named Endpoint or the start or end of
///   a named Connection
/// - a collection of named 'groups', specified by sequences of named
///   Connections
///
/// Parsing will fail if there is no way to concretely resolve all of the
/// Endpoint references, e.g., if a document specifies that Connection-A
/// is an arc starting at the end of Connection-B and that Connection-B
/// is an arc starting at the end of Connection-A.  All referential chains
/// must bottom out in explicitly-named Endpoints.
///
/// @subsubsection general_considerations General considerations
///
/// All the road geometry information must be under a root node called
/// `maliput_multilane_builder`, otherwise it will not be parsed.
///
/// <h4>Units</h4>
///
/// The following list shows the expected units for floating-point quantities:
///
/// - Positions, distances and lengths: meters [m].
/// - Angles: degrees [°] (no minutes nor seconds, just degrees).
/// - Derivatives of positions: meters per meter [m/m] \(i.e., a unitless
///   slope).
/// - Derivatives of angles: degrees per meter [°/m].
///
/// All positions, distances, lengths, angles and derivatives are floating point
/// numbers. Other type of quantities will be integers.
///
/// <h4>Miscellaneous</h4>
///
/// Clarifications to better understand the nomenclature used within this
/// description:
///
/// - In code snippets, strings in capital letters represent values that the
///   YAML writer must choose and the others are keywords to be parsed.
/// - When referring to keywords in the YAML, `non-capitalized` strings will
///   be used.
/// - When referring to types within `maliput`, `Capitalized` strings will be
///   used.
///
/// <h4>Coordinates and frames</h4>
///
/// For points in space, a right handed, orthonormal and inertial ℝ³ frame is
/// used. Its basis is (x̂, ŷ, ẑ), where x̂, ŷ are coplanar with the ground and ẑ
/// points upwards, and positions are expressed as `(x, y, z)` triples.
/// Also, the Θ angle rotating around the ẑ axis is used to define
/// headings. These rotations are right handed and an angle of 0° points in
/// the x̂ direction. Angles with respect to a plane parallel to `z = 0` can
/// be defined. A heading vector pointing the direction of the lane at that
/// point is used as rotation axis and the angle is clockwise. Those will
/// express superelevation.
///
/// @subsubsection example_of_general_structure Example of General Structure
///
/// Below you can see a snippet with the general YAML structure.
///
/// @code{.yml}
///
/// maliput_multilane_builder:
///  id: "my_road_geometry"
///  lane_width: 3.2
///  left_shoulder: 1.25
///  right_shoulder: 2.47
///  elevation_bounds: [0., 7.6]
///  scale_length: 1.
///  linear_tolerance: 0.1
///  angular_tolerance: 0.1
///  computation_policy: prefer-accuracy
///  points:
///    point_a:
///      xypoint: [0, 0, 0]
///      zpoint: [0, 0, 0, 0]
///    point_b:
///      xypoint: [50, 5, 0]
///      zpoint: [0, 0, 0]
///    ...
///  connections:
///    conn_a:
///       left_shoulder: 1.3 # Optional
///       lanes: [3, 2, -5.3]
///       start: ["lane.1", "points.point_a.forward"]
///       arc: [30.25, -45]
///       z_end: ["lane.0", [0, 3, 30, 3.1]]
///    conn_b:
///    ...
///  groups:
///    group_A: [conn_a, conn_b]
///
/// @endcode
///
/// @subsubsection entities Entities
///
/// <h4>`maliput_multilane_builder`</h4>
///
/// `maliput_multilane_builder` holds all the common and default configurations
/// to build a `RoadGeometry`. All of them, except `groups`, must be defined
/// though some of them may be empty.
///
/// It will be represented as a mapping:
///
/// @code{.yml}
///
/// maliput_multilane_builder:
///  id: "ID"
///  lane_width: LW
///  left_shoulder: LS
///  right_shoulder: RS
///  elevation_bounds: [EB_MIN, EB_MAX]
///  linear_tolerance: LT
///  angular_tolerance: AT
///  scale_length: SL
///  computation_policy: CP
///  points:
///    ...
///  connections:
///    ...
///  groups:
///    ...
///
/// @endcode
///
/// Where:
///
/// - _ID_ is a string scalar that defines the _ID_ of the `RoadGeometry`.
/// - _LW_ is the width of the lanes. Lane’s centerline will be placed at
///   the same lane width distance one from the other. It must be non negative.
/// - _LS_ and _RS_ are default left and right shoulders are extra spaces
///   added to the right of the last lane and left to the first lane
///   respectively.
///   Their purpose is to increase segment bounds. Both must be non negative.
/// - _EB\_MIN_ and _EB\_MAX_ define minimum and maximum height values of the
///   road’s volume. The minimum value must be non positive, thus the maximum
///   must be non negative.
/// - _LT_ and _AT_ are position and orientation tolerances which are non
///   negative numbers that define the error of mapping a world coordinate or
///   orientation in a custom lane-frame.
/// - _SL_ is the minimum spatial period of variation in `connections`'
///   reference curve.
/// - A _CP_ label, which could either be `prefer-accuracy` or `prefer-speed`
///   The former guides the computations to be as accurate as precision states.
///   The latter will be accurate whenever possible, but it's not guaranteed in
///   favor of faster computations.
/// - _points_ is a map of `endpoint`s to build `connection`s. At least one
///   point is required to anchor the connections to world-frame.
/// - _connections_ is a map that holds all the `connection` definitions. It
///   may be empty if no `Connection` is going to be defined.
/// - _groups_ is a map of `groups` where `connections` can be put together.
///   It may be empty or not defined if no group is going to be made.
///
///
/// <h4>`points`</h4>
///
/// A collection of points in 3D space. Each one will be under a tag (used to
/// reference it within `connection` description) and defined by an
/// `endpoint_xy` and a `endpoint_z`. Both sequences must be provided.
///
/// It will be represented as a mapping like:
///
/// @code{.yml}
///
/// endpoint:
///   xypoint: [X, Y, THETHA]
///   zpoint: [Z, Z_DOT, THETA, THETA_DOT]
///
/// @endcode
///
/// Where:
///
/// - `xypoint` is the `endpoint_xy` sequence.
/// - `zpoint` is the `endpoint_z` sequence.
///
/// <h4>`endpoint_xy`</h4>
///
/// A point in the plane `z = 0` expressed as `(x, y, Θ)`, where
/// `(x, y)` defines the position and Θ defines the heading angle of
/// the endpoint. All coordinates must be provided.
///
/// It will be represented as a sequence:
///
/// @code{.yml}
///
/// endpoint_xy: [X, Y, THETA]
///
/// @endcode
///
/// Where:
///
/// - _X_ is the `x` coordinate.
/// - _Y_ is the `y` coordinate.
/// - _THETA_ is the Θ coordinate.
///
/// <h4>`endpoint_z`</h4>
///
/// Specifies elevation, slope, superelevation and its speed of change at a
/// point over the plane `z = 0` as `(z, z', Θ, Θ')`, where `z` and
/// Θ' are the elevation and superelevation of the road at the endpoint
/// and `z'` and Θ' are their respective derivatives with respect to
/// an arc-length coordinate `t` that travels along curve’s projection over
/// the `z = 0` plane. All coordinates must be provided.
///
/// It will be represented as a sequence:
///
/// @code{.yml}
///
/// endpoint_z: [Z, Z_DOT, THETA, THETA_DOT]
///
/// @endcode
///
/// Where:
///
/// - _Z_ is the `z` coordinate.
/// - _Z\_DOT_ is the `z′` coordinate.
/// - _THETA_ is the Θ coordinate.
/// - _THETA\_DOT_ is the Θ′ coordinate. This parameter is optional, and
///   typically should be omitted. When omitted, this value will be
///   automatically calculated such that _G1_ continuity of the road surface is
///   preserved.
///
/// <h4>`connections`</h4>
///
/// A `connection` defines a `Segment` and must provide the number of lanes,
/// start `endpoint` and end `endpoint_z` information. Either line `length` or
/// `arc` must be provided to define the planar geometry of that `connection`.
/// Optional extra information can also be provided and it will modify the way
/// the `connection` will be created. `connections` is a collection of
/// `connection`s and those will be identified by their tag. Each tag will name
/// a `connection`, can be referenced by other `connection`s and to create
/// `group`s, and will be used as `Segment`'s ID as well.
///
/// `start` `endpoint` and `end` `endpoint_z` can either refer to a reference
/// road curve or to the lane start and end `Endpoint`s. When only `start` and
/// `z_end` or `explicit_end` are provided, those will refer to the reference
/// road curve of the `Connection`. `r_ref` can be provided to state the offset
/// distance from the reference road curve to `ref_lane` `connection`’s lane.
/// Lanes are 0-indexed. In addition, left and right shoulder distances can be
/// provided and those will override default values. `left_shoulder` and
/// `right_shoulder` must be bigger or equal to zero if provided.
///
/// Sample line-connections mapping are shown below:
///
/// - Example 1: reference curve from points.
///
/// @code{.yml}
///
/// CONNECTION_NAME:
///   left_shoulder: LS
///   right_shoulder: RS
///   lanes: [NL, NREF, RREF]
///   start: ["ref", "points.POINT_NAME_1.(forward|reverse)"]
///   length: L
///   z_end: ["ref", [Z, Z_DOT, THETA, THETA_DOT]]
///   # The following can be used instead of z_end:
///   # explicit_end: ["ref", "points.POINT_NAME_2.(forward|reverse)"]
///
/// @endcode
///
/// Within `z_end`, _THETA\_DOT_ is optional, and typically should be omitted.
/// When omitted, this value will be automatically calculated such that _G1_
/// continuity of the road surface is preserved. Otherwise, provided
/// _THETA\_DOT_ will be used and the
/// @ref maliput::multilane::Builder "Builder" will check whether or not
///  _G1_ is preserved.
///
/// When `explicit_end` is used, _THETA\_DOT_ will be set by
/// @ref maliput::multilane::Builder "Builder" to preserve _G1_ road
/// surface continuity.
///
/// - Example 2: reference curve from connections.
///
/// @code{.yml}
///
/// CONNECTION_NAME:
///   left_shoulder: LS
///   right_shoulder: RS
///   lanes: [NL, NREF, RREF]
///   start: [
///     "ref",
///     "connections.CONN_NAME_1.(start|end).ref.(forward|reverse)"
///   ]
///   length: L
///   explicit_end: [
///     "ref",
///     "connections.CONN_NAME_2.(start|end).ref.(forward|reverse)"
///   ]
///   # The following can be used instead of explicit_end:
///   # z_end: ["ref", [Z, Z_DOT, THETA, THETA_DOT]]
///
/// @endcode
///
/// Within `z_end`, _THETA\_DOT_ is optional, and typically should be omitted.
/// When omitted, this value will be automatically calculated such that _G1_
/// continuity of the road surface is preserved. Otherwise, provided
/// _THETA\_DOT_ will be used and the
/// @ref maliput::multilane::Builder "Builder" will check whether or not
/// _G1_ is preserved.
///
/// When `explicit_end` is used, _THETA\_DOT_ will be set by
/// @ref maliput::multilane::Builder "Builder" to preserve _G1_
/// continuity of the road surface.
///
/// - Example 3: lane curve from points.
///
/// @code{.yml}
///
/// CONNECTION_NAME:
///   left_shoulder: LS
///   right_shoulder: RS
///   lanes: [NL, NREF, RREF]
///   start: ["lane.LN_1", "points.POINT_NAME_1.(forward|reverse)"]
///   length: L
///   z_end: ["lane.LN_2", [Z, Z_DOT, THETA]]
///   # The following can be used instead of z_end:
///   # explicit_end: ["lane.LN_2", "points.POINT_NAME_2.(forward|reverse)"]
///
/// @endcode
///
/// None of the lane-based flavors allow to have _THETA\_DOT_ at either
/// `start` or `z_end`. @ref maliput::multilane::Builder "Builder" will
/// adjust them to preserve _G1_ continuity of the road surface.
///
/// - Example 4: lane curve from other connections' lane curves.
///
/// @code{.yml}
///
/// CONNECTION_NAME:
///   left_shoulder: LS
///   right_shoulder: RS
///   lanes: [NL, NREF, RREF]
///   start: [
///     "lane.LN_1",
///     "connections.CONN_NAME_1.(start|end).LN_2.(forward|reverse)"
///   ]
///   length: L
///   explicit_end: [
///     "lane.LN_2",
///     "connections.CONN_NAME_2.(start|end).LN_4.(forward|reverse)"
///   ]
///   # The following can be used instead of explicit_end:
///   # z_end: ["lane.LN_2", [Z, Z_DOT, THETA]]
///
/// @endcode
///
/// None of the lane-based flavors allow to have _THETA\_DOT_ at either
/// `start` or `z_end`. @ref maliput::multilane::Builder "Builder" will
/// adjust them to preserve _G1_ continuity of the road surface.
///
/// From examples above:
///
/// - `lanes` holds number of lanes, reference lane and distance from the
///   reference lane to the reference curve.
/// - `left_shoulder` is the extra space at the right side of the last lane
///   of the connection. It will override default values.
/// - `right_shoulder` is the extra space at the left side of the first lane
///   of the `connection`. It will override default values.
/// - `start` is used to define the start `endpoint` of one the `connection`’s
///   curves. It may have multiple options. Those can be split into two
///   elements:
///   - The first element could be:
///     -# `ref` to point the reference curve.
///     -# `lane.LN` to point a specific lane.\n
///   - The second element is composed of one of the following options:
///     -# A reference to an `endpoint` in the points collection. Either
///        `forward` or `reverse` should be used to indicate the direction of
///        the `endpoint`.
///     -# The start or end `endpoint` of a `connection`’s reference curve or
///        lane. Either forward or reverse should be used to indicate the
///        direction of the `endpoint`. When using the forward the `endpoint`
///        will be used as is. Otherwise (using `reverse`) the `Endpoint` will
///        be reversed.
/// - `length` is the `connection`’s reference road curve planar line length.
/// - `arc` is the `connection`’s reference curve planar piece of arc.
/// - `z_end` is the `endpoint_z` information to end one of the `connection`’s
///    curves. It is composed of two elements too. The first one points to the
///    reference curve when `ref` is present. Otherwise, `lane.LN` must be
///    specified.
/// - `explicit_end` is a node similar to `start`. It is composed of two
///    parts. The first one points to the reference curve when `ref` is present.
///    Otherwise, `lane.LN` must be specified. The second part is used to point
///    the `endpoint` information which could be provided by a `connection` or
///    the `points` collection. When using a `connection`, two options are
///    available: the reference curve or a lane.
///
/// Possible combinations to define a `connection` node are:
///
/// - Each `connection` must have either `length` or `arc`.
/// - Each `connection` must have either `z_end` or `explicit_end`.
/// - `start` and `explicit_end` possible combinations:
///   -# This LANE from other LANE.
///   -# This REF from other REF.
///   -# This LANE from POINT.
///   -# This REF from POINT.
///
/// At least one connection must _start_ with "LANE from POINT" or "REF from
/// POINT" in order to anchor the road geometry in the world frame.
///
/// <h4>`arc`</h4>
///
/// Constant radius arcs are defined in terms of a radius and an angle span.
/// `arc`s are used to define planar curves on the the `z = 0` plane,
/// starting from an `endpoint_xy`. Radius must be positive, and arc's center
/// would be to the left (i.e. rotating +90° start `endpoint_xy`’s heading
/// vector) when theta is positive, otherwise it would be to the right (i.e.
/// rotating -90° start `endpoint_xy`’s heading).
///
/// It will be represented as a sequence:
///
/// @code{.yml}
///
/// arc: [RADIUS, THETA]
///
/// @endcode
///
/// Where:
///
/// - _RADIUS_ is the radius of the `arc`.
/// - _THETA_ is the angle span of the `arc`.
///
/// <h4>`groups`</h4>
///
/// A group specifies a set of connections whose Segments will be placed
/// together in the same `Junction`. A `connection` may only belong to a single
/// `group`. If a `connection` is not in any `group`, its `Segment` will receive
/// its own `Junction`.
///
/// It will be represented as a mapping:
///
/// @code{.yml}
///
/// GROUP_NAME: [C_1, C_2, C_3]
///
/// @endcode
///
/// Where:
///
/// - `C_1`, `C_2`, `C_3` are `connections`’ IDs.
