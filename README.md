# Frappy Framework

current running code at SINQ, with newest changes not yet pushed
through the Gerrit workflow at MLZ

## Branches

branches:

- mlz: master from forge.frm2.tum.de:29418/sine2020/secop/playground
  this is not present at git.psi.ch:sinqdev/frappy.git!
- master: the last synced state between mlz and wip/work, except an added README.md
  (this does NOT contain local repo files only, however, all common files work/mlz should match)
- core: the modifications of the core frappy parts (to be going through gerrit)
- work: current working version, usually in use on /home/l_samenv/frappy (and on neutron instruments)
  this should be a copy of an earlier state of the wip branch
- wip: current test version, usually in use on /home/l_samenv/frappy_wip

IMPORTANT: make commits containing either only files to be pushed to Gerrit or only
PSI internal files, not mixed. Mark local commits with '[PSI]' in the commit message.


master --> mlz  # these branches match after a sync step, but they might have a different history
master --> work --> wip

apply commits from mlz to master: (rebase ?) or use cherry-pick:

   git cherry-pick <sha1>..<sha2>

where sha1 is the last commit already in wip, and sha2 ist the last commit to be applied
(for a single commit <sha1>.. may be omitted)

the wip branch is also present in an other directory (currently zolliker/switchdrive/gitmlz/frappy),
where commits may be cherry picked for input to Gerrit. As generally in the review process some additional
changes are done, eventually a sync step should happen:

1) ideally, this is done when work and wip match
2) make sure branches mlz, master, wip and work are in synv with remote, push/pull otherwise
3) cherry-pick commits from mlz to master
4) make sure master and mlz branches match (git diff --name-only master..wip should only return README.md)
5) create branch new_work from master
6) go through commits in wip and sort out:
   - core commits already pushed through gerrit are skipped
   - all other commits are to be cherry-picked
7) when arrived at the point where the new working version should be,
   copy new_wip branch to work with 'git checkout work;git checkout new_wip .'
   (note the dot!) and then commit this.
8) continue with (6) if wip and work should differ
9) do like (7), but for wip branch
10) delete new_wip branch, push master, wip and work branches


## Procedure to update PPMS

1) git checkout wip (or work, whatever state to copy to ppms)
2) git checkout -B ppms   # local branch ?
3) assume PPMSData is mounted on /Volumes/PPMSData

   cp -r secop_psi /Volumes/PPMSData/zolliker/frappy/secop_psi
   cp -r secop /Volumes/PPMSData/zolliker/frappy/secop

   it may be that additional folder have to copied ...

