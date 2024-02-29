        if score > self.min_score:
            print(f'Not over min score {next_cost = }, {cap = }, {node = }, {score = } {path = }')
            return (path, float('inf'))