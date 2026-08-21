#based on starting code from Gemini
import AbstractAlgebra: evaluate
using AlgebraicSolving

function local_system(sys::Vector,vars_to_use::Vector,R_global::MPolyRing)
    local_var_names = [string(v) for v in vars_to_use]
    R_local, vars_local = QQ[local_var_names...]
    
    # 2. Build Global -> Local evaluation vector (Length = nvars(R_global))
    g_vars = AbstractAlgebra.gens(R_global)
    vals_to_local = [R_local(0) for _ in 1:nvars(R_global)]
    
    for (i, v_glob) in enumerate(vars_to_use)
       g_idx = findfirst(==(v_glob), g_vars)
       vals_to_local[g_idx] = vars_local[i]
    end
    
    # Map global polynomials into the local ring
    sys_local = Ideal([evaluate(p, vals_to_local) for p in sys])
    return sys_local,R_local,vars_local
end
# Helper: Perform elimination in a small local sub-ring
function eliminate_subsystem(sys::Vector, vars_to_elim::Vector, vars_to_keep::Vector, R_global::MPolyRing)
    all_active_global = vcat(vars_to_elim, vars_to_keep)
    
    # 1. Define local sub-ring (eliminated variables listed first)
    local_var_names = [string(v) for v in all_active_global]
    R_local, vars_local = QQ[local_var_names...]
    
    # 2. Build Global -> Local evaluation vector (Length = nvars(R_global))
    g_vars = AbstractAlgebra.gens(R_global)
    vals_to_local = [R_local(0) for _ in 1:nvars(R_global)]
    
    for (i, v_glob) in enumerate(all_active_global)
       g_idx = findfirst(==(v_glob), g_vars)
       vals_to_local[g_idx] = vars_local[i]
    end
    
    # Map global polynomials into the local ring
    sys_local = Ideal([evaluate(p, vals_to_local) for p in sys])
    
    # 3. Compute Gröbner basis in local ring
    gb_local = groebner_basis(sys_local)
    
    # 4. Filter out polynomials containing eliminated variables
    elim_local_vars = vars_local[1:length(vars_to_elim)]
    reduced_local = filter(p -> isempty(intersect(vars(p), elim_local_vars)), gb_local)
    
    # 5. Map back to Global ring
    # length(all_active_global) matches nvars(R_local)
    return [evaluate(p, all_active_global) for p in reduced_local]
end

function forward_pass_step(accumulated::MPolyRingElem[],eqn_block,vars_to_elim,vars_to_keep,R_global::MPolyRing)
    # Combine accumulated constraints with current block's equations
    current_sys = vcat(accumulated, eqn_block)
    return eliminate_subsystem(current_sys, vars_to_elim, vars_to_keep,R_global)
end
# 2. Sequential Forward Cascade
acc_constraints = MPolyRingElem[] # Accumulated constraints passed down the chain

for k in 1:3
    # Combine accumulated constraints with current block's equations
    current_sys = vcat(acc_constraints, F_blocks[k])
    
    # Variables to eliminate: block k internal + left coupling variables
    # Variables to keep: coupling variables connecting to block k+1
    elim_vars = blocks[k]
    keep_vars = blocks[k+1]
    
    # Run elimination step locally
    acc_constraints = eliminate_subsystem(current_sys, elim_vars, keep_vars)
    println("Stage $k complete: $(length(acc_constraints)) boundary equations propagated.")
end

# 3. Final Solve on the Last Block
final_sys = vcat(acc_constraints, F_blocks[4])
solutions_block4 = solve(final_sys)

# 4. Back-Substitution (Solve remaining blocks in reverse order using solutions_block4)