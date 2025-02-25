# automotive-simulator

## How to work with the automotive-core git submodule

Clone the project

```
git clone --recurse-submodules <url>
```

Fetch changes from the remote and the submodule

```
git pull --recurse-submodules
```

Fetch changes from the submodule only

```
git submodule update --remote
```

Push changes to remote and check if all submodle changes have been pushed as well

```
git push --recurse-submodules=check
```

Push changes to remote and submodules at the same time

```
git push --recurse-submodules=on-demand
```