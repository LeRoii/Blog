### install redis on mac
```
brew install redis

ln -f /usr/local/Cellar/redis/3.0.4/homebrew.mxcl.redis.plist ~/Library/LaunchAgents/  

launchctl load ~/Library/LaunchAgents/homebrew.mxcl.redis.plist 
```