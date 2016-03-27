---
layout: post
title:  "Similarity with inverted index in Redis"
date:   2016-03-27 13:42:41 +0100
categories: 
---

One of the most common measure of similarity between two users that have a vector representation of the likes and dislikes, is to count the amount of items they have in common an apply some formula. But this method requires to iterate over all users in order to rank them. Another solution is to build an inverted index with the liked items, so we can search over all users that have at least one element in common, this way in most cases we can reduce the number of users over which we have to iterate.

We are going to use Redis, because it allows us to perform all operations in memory and it's very fast in doing set operations like unions or intersections.

<h2>Update data</h2>
 
 Here I'm going to assume that the items will be binary, so when we delete something, we can delete it from the set of items, and we don't need to mantain a counter.
 
 We need to implement two functions Add(user, items), which adds the items to user, and the complementary function Delete(user, items)

{% highlight python %}

def add(conn, user, items):

    # We use a pipeline to send everything together
    pipeline=conn.pipeline()

    # User set of items
    pipeline.sadd("USER_ITEMS:"+str(user), items)

    # Inverted index
    for elem in items:
        pipeline.sadd("ITEM_USERS:"+str(elem), user)

    pipeline.execute()

def remove(conn, user, items):

    # We use a pipeline to send everything together
    pipeline=conn.pipeline()

    # User set of items
    pipeline.srem("USER_ITEMS:"+str(user), items)

    # Inverted index
    for elem in items:
        pipeline.srem("ITEM_USERS:"+str(elem), user)

    pipeline.execute()

{% endhighlight %}

I have used a pipeline to send everything to the server together.

<h2>Query data</h2>

As a query we are going to define the function Query(user, n_users) which returns the most similar n_users to user. The algorithm to do this will be to first do an union of all the sets of users in the inverted index of the items of the user we are querying, and then iterate over this set to find the n most similar users. Because the set of possible users can still be very large, I'm going to use a lua script to execute everything in the redis server (<a href="http://oldblog.antirez.com/post/redis-and-scripting.html">http://oldblog.antirez.com/post/redis-and-scripting.html</a>). 

{% highlight lua %}
local user_key = KEYS[1]..ARGV[1]

local results = redis.call("SSCAN", user_key, 0, "match", "*")

local t = { }
local size = table.getn(results[2])

for i=1,size,1 do
  t[#t+1] = tostring(KEYS[2] .. results[2][i])
end 

local users = redis.call("SUNION", unpack(t) )

size = table.getn(users)

local v = {}
local result = {}
for i=1,size,1 do
  result = redis.call("SINTER", user_key, KEYS[1]..users[i] )
  table.insert(v, {-table.getn(result), i} )
end 

local function compare(a,b)
  return a[1] < b[1]
end

table.sort(v, compare)

local result = {}
for i,n in ipairs(v) do 
  if i <= tonumber(ARGV[2]) then
    table.insert(result, users[n[2]])
  else
    break
  end
end

return result
{% endhighlight %}

And the python part would look something like this

{% highlight python %}
def script_load(script):
    sha = [None]
    def call(conn, keys=[], args=[], force_eval=False):
        if not force_eval:
            if not sha[0]:
                sha[0] = conn.execute_command(
                            "SCRIPT", "LOAD", script, parse="LOAD")

            try:
                return conn.execute_command(
                           "EVALSHA", sha[0], len(keys), *(keys+args))
            except redis.exceptions.ResponseError as msg:
                if not msg.args[0].startswith("NOSCRIPT"):
                    raise
        return conn.execute_command(
            "EVAL", script, len(keys), *(keys+args))

    return call
    
def query(user, n_users):
    f = open("RedisQuery.lua", "r")
    query_script = script_load(f.read())
    result = query_script(r, ["USER_ITEMS:", "ITEM_USERS:"], [user, n_users])
{% endhighlight %}

This will return the most similar users. Obviously, this would still be slow in some cases, so it should be computed offline, or maybe there could be some heuristic, for example, location-based, to reduce the search size.
